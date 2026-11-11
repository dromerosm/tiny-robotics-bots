// track: see track.h.
//
// One task, the finder, on core 1. It owns the camera and the face model:
// it starts them when the face task wants to follow, and stops and frees
// them for a talk or a recording. The echo canceller needs the same core,
// and a talk needs the internal RAM, so only one of the two runs at a time.
//
// The face task calls track_step() every frame. It reads what the finder
// found and moves the head: toward the face, or to a new spot when there
// has been no face for a while. The rules and numbers are 11-track's. With
// the camera view open (track_show), it also shows each frame with a box on
// each face (track_frame).
#include "track.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "face.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "tr_board.h"
#include "tr_screen.h"
#include "tr_web.h"

static const char *TAG = "track";

#define IDLE_PAUSE_US     (20 * 1000 * 1000) // after a move from the page
#define FOLLOW_DEAD       0.15f              // no move while the face is this close to the center
#define FOLLOW_YAW        18.0f // degrees of yaw from the center to the edge of the frame (a guess)
#define FOLLOW_PITCH      12.0f // the same for pitch (a guess)
#define MS_PER_DEG        25    // move time per degree: about 40 degrees per second
#define MOVE_MIN_MS       150
#define MOVE_MAX_MS       600
#define YAW_SIGN          1.0f          // flip if the head turns away from the face
#define PITCH_SIGN        (-1.0f)       // flip if the head looks away up or down
#define LOST_US           (1000 * 1000) // no face for this long: the face is lost
#define RELEASE_US        (2000 * 1000) // torque off after this long with no move
#define SEARCH_AFTER_US   (5000 * 1000) // no face for this long: start searching
#define SEARCH_YAW        70            // looks stay within +-70 degrees of yaw
#define SEARCH_PITCH_LOW  5             // and 5 to 40 degrees of pitch
#define SEARCH_PITCH_HIGH 40
#define SEARCH_MIN_YAW    33            // a new look turns at least this far
#define SEARCH_MIN_PITCH  26            // or tilts at least this far
#define SEARCH_MAX_STEP   60            // but no look turns more than this
#define SEARCH_MS_PER_DEG 50            // looks turn at about 20 degrees per second
#define SEARCH_STAY_US    (3000 * 1000) // then it stays still this long
#define SLOW_FIND_MS      400           // with no face: about 2 finds per second
#define MOVE_MIN_MV       3600          // no head moves on a nearly empty battery
#define HOME_YAW          0.0f
#define HOME_PITCH        5.0f
#define HOME_MS           1000
#define CENTER_FG         TR_LCD_RGB(90, 160, 255) // the center and dead zone in the camera view
// After track_quiet(), track_step() waits this long before it may start
// again: by then the face task has seen that a talk or a recording is on.
#define HOLD_US (500 * 1000)

// What the finder found last, for the face task.
typedef struct {
    uint32_t seq; // one more for each find
    int n, w, h;  // faces, and the frame size
    int cx, cy;   // the center of the biggest face
} target_t;
static portMUX_TYPE target_lock = portMUX_INITIALIZER_UNLOCKED;
static target_t target;

// The camera view: with it open, the finder hands each frame and its boxes
// to the face task, which shows them and gives the frame back to the
// camera. When the face task is busy, the finder gives the frame back
// itself, so the screen never slows the finder.
typedef struct {
    camera_fb_t *fb;
    int n;
    face_box_t boxes[FACE_MAX];
} found_frame_t;
static QueueHandle_t to_show; // one frame for the screen
static volatile bool show;    // the camera view is open
// showing: the face task has a frame. stopping: the finder is about to stop
// the camera, so the face task takes no new frame.
static portMUX_TYPE show_lock = portMUX_INITIALIZER_UNLOCKED;
static bool showing, stopping;
static volatile int show_fps10; // frames drawn on the screen per second, x 10

static TaskHandle_t finder;
static SemaphoreHandle_t head_lock; // the servos: track_step() or track_quiet()
static volatile bool want;          // the finder should run
static volatile bool loaded;        // the camera and the model are up
static volatile bool failed;        // no camera or no memory: don't try again
static volatile bool idle;          // the finder waits, with the camera and the model off
static bool ever_loaded;            // only the finder uses it
static volatile int64_t hold_until;
static volatile int faces, find_ms, fps10;
static volatile float err_x, err_y;

// Only the face task uses these (in track_step).
static bool homed, following;
static uint32_t seen_seq;
static int64_t last_seen, since, next_look, release_at;
static float follow_yaw, follow_pitch;

static void log_memory(const char *what, int64_t t0)
{
    ESP_LOGI(TAG, "%s in %lld ms; free: internal %u KB (biggest block %u KB), PSRAM %u KB", what,
             (esp_timer_get_time() - t0) / 1000,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024,
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024,
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024);
}

static void finder_task(void *arg)
{
    int finds = 0, ms_sum = 0;
    int64_t window = 0, face_at = -1;
    for (;;) {
        if (!want) {
            if (loaded) {
                const int64_t t0 = esp_timer_get_time();
                // No frame may be out when the camera stops: wait for the
                // one on the screen, and take back the one in the queue.
                taskENTER_CRITICAL(&show_lock);
                stopping = true;
                taskEXIT_CRITICAL(&show_lock);
                for (;;) {
                    taskENTER_CRITICAL(&show_lock);
                    const bool out = showing;
                    taskEXIT_CRITICAL(&show_lock);
                    if (!out) {
                        break;
                    }
                    vTaskDelay(1);
                }
                found_frame_t f;
                if (xQueueReceive(to_show, &f, 0) == pdTRUE) {
                    esp_camera_fb_return(f.fb);
                }
                face_stop();
                tr_cam_stop();
                taskENTER_CRITICAL(&show_lock);
                stopping = false;
                taskEXIT_CRITICAL(&show_lock);
                faces = 0;
                loaded = false;
                log_memory("camera and model off", t0);
            }
            idle = true;
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            idle = false;
            continue;
        }
        if (!loaded) {
            const int64_t t0 = esp_timer_get_time();
            // A camera that never started (as in the emulator) is not tried
            // again. One that worked before gets another try in 10 s.
            const bool cam = tr_cam_start() == ESP_OK;
            if (!cam || face_start() != ESP_OK) {
                ESP_LOGW(TAG, "%s: the head does not follow%s",
                         cam ? "no memory for the face model" : "no camera",
                         ever_loaded ? " for 10 s" : "");
                if (cam) {
                    tr_cam_stop();
                }
                failed = !ever_loaded;
                hold_until = esp_timer_get_time() + 10 * 1000 * 1000;
                want = false;
                continue;
            }
            loaded = ever_loaded = true;
            finds = ms_sum = 0;
            window = esp_timer_get_time();
            face_at = -1;
            log_memory("camera and model on", t0);
        }
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            continue; // the camera timed out; try again
        }
        found_frame_t f = {.fb = fb};
        face_box_t *boxes = f.boxes;
        const int64_t t0 = esp_timer_get_time();
        const int n = f.n = face_find(fb->buf, fb->width, fb->height, boxes, FACE_MAX);
        const int ms = (esp_timer_get_time() - t0) / 1000;
        taskENTER_CRITICAL(&target_lock);
        target.seq++;
        target.n = n;
        target.w = fb->width;
        target.h = fb->height;
        if (n > 0) {
            target.cx = (boxes[0].x0 + boxes[0].x1) / 2;
            target.cy = (boxes[0].y0 + boxes[0].y1) / 2;
        }
        taskEXIT_CRITICAL(&target_lock);
        // To the screen if the view is open and free; if not, back to the camera.
        if (!show || xQueueSend(to_show, &f, 0) != pdTRUE) {
            esp_camera_fb_return(fb);
        }
        faces = n;
        finds++;
        ms_sum += ms;
        const int64_t now = esp_timer_get_time();
        if (n > 0) {
            face_at = now;
        }
        if (now - window >= 10000000) { // every 10 s: /log holds only 4 KB
            find_ms = ms_sum / finds;
            fps10 = (int)(finds * 10000000LL / (now - window));
            ESP_LOGI(TAG, "%d face(s), find %d ms, %d.%d finds/s, finder stack left %u B", n,
                     find_ms, fps10 / 10, fps10 % 10, (unsigned)uxTaskGetStackHighWaterMark(NULL));
            finds = ms_sum = 0;
            window = now;
        }
        // No face for a while and the camera view closed: a break. A notify
        // ends it at once.
        if (!show && (face_at < 0 || now - face_at > LOST_US)) {
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(SLOW_FIND_MS));
        }
    }
}

esp_err_t track_init(void)
{
    head_lock = xSemaphoreCreateMutex();
    to_show = xQueueCreate(1, sizeof(found_frame_t));
    if (!head_lock || !to_show ||
        // Its stack in PSRAM: the model's data is there too, and internal
        // RAM is short (the echo canceller keeps 136 KB of it).
        xTaskCreatePinnedToCoreWithCaps(finder_task, "finder", 6144, NULL, 4, &finder, 1,
                                        MALLOC_CAP_SPIRAM) != pdPASS) {
        finder = NULL;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void track_quiet(void)
{
    if (!finder) {
        return;
    }
    xSemaphoreTake(head_lock, portMAX_DELAY);
    hold_until = esp_timer_get_time() + HOLD_US;
    want = false;
    xTaskNotifyGive(finder);
    // The finder ends its frame first, or its start of the camera.
    for (int i = 0; i < 300 && !(idle && !loaded); i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (loaded || !idle) {
        ESP_LOGW(TAG, "the camera did not stop in 3 s");
    }
    if (tr_web_servos()) {
        tr_web_head_off();
    }
    err_x = err_y = 0;
    xSemaphoreGive(head_lock);
}

// True if the battery can take a head move: USB power, or the battery at
// MOVE_MIN_MV or more. Read at most once a second.
static bool power_for_moves(int64_t now)
{
    static int64_t next_check;
    static bool ok = true;
    if (now >= next_check) {
        next_check = now + 1000000;
        tr_power_t p;
        tr_battery_t b;
        const bool usb = tr_power_read(&p) == ESP_OK && p.usb;
        const bool now_ok = usb || tr_battery_read(&b) != ESP_OK || b.millivolts >= MOVE_MIN_MV;
        if (now_ok != ok) {
            ok = now_ok;
            if (ok) {
                ESP_LOGI(TAG, "battery ok again: the head moves");
            } else {
                ESP_LOGW(TAG, "battery below %d mV: the head stays still", MOVE_MIN_MV);
            }
        }
    }
    return ok;
}

static bool page_moved(int64_t now)
{
    const int64_t last_move = tr_web_last_move_us();
    return last_move >= 0 && now - last_move < IDLE_PAUSE_US;
}

// Move time for a turn of this many degrees: about the same speed for small
// and big turns.
static int move_ms(float degrees)
{
    const int ms = (int)(fabsf(degrees) * MS_PER_DEG);
    return ms < MOVE_MIN_MS ? MOVE_MIN_MS : ms > MOVE_MAX_MS ? MOVE_MAX_MS : ms;
}

// Turn toward the face. True if the head moved. With move false it only
// keeps track of the face (for the eyes), and the head stays still.
static bool follow_step(int64_t now, bool move)
{
    taskENTER_CRITICAL(&target_lock);
    const target_t t = target;
    taskEXIT_CRITICAL(&target_lock);
    bool moved = false;
    if (t.seq != seen_seq) {
        seen_seq = t.seq;
        if (t.n > 0 && t.w > 0 && t.h > 0) {
            following = true;
            last_seen = now;
            const float ex = (t.cx - t.w / 2.0f) / (t.w / 2.0f);
            const float ey = (t.cy - t.h / 2.0f) / (t.h / 2.0f);
            err_x = ex;
            err_y = ey;
            if (move && !page_moved(now) && tr_web_servos() && power_for_moves(now) &&
                (fabsf(ex) > FOLLOW_DEAD || fabsf(ey) > FOLLOW_DEAD) &&
                tr_web_head_power() == ESP_OK) {
                tr_servo_state_t st;
                if (fabsf(ex) > FOLLOW_DEAD) {
                    if (tr_servo_read(TR_SERVO_YAW, &st) == ESP_OK) {
                        follow_yaw = st.degrees;
                    }
                    const float goal =
                        fminf(fmaxf(follow_yaw + YAW_SIGN * ex * FOLLOW_YAW, -80.0f), 80.0f);
                    tr_servo_move(TR_SERVO_YAW, goal, move_ms(goal - follow_yaw));
                    follow_yaw = goal;
                }
                if (fabsf(ey) > FOLLOW_DEAD) {
                    if (tr_servo_read(TR_SERVO_PITCH, &st) == ESP_OK) {
                        follow_pitch = st.degrees;
                    }
                    const float goal =
                        fminf(fmaxf(follow_pitch + PITCH_SIGN * ey * FOLLOW_PITCH, TR_PITCH_MIN),
                              TR_PITCH_MAX);
                    tr_servo_move(TR_SERVO_PITCH, goal, move_ms(goal - follow_pitch));
                    follow_pitch = goal;
                }
                moved = true;
            }
        }
    }
    if (following && now - last_seen > LOST_US) {
        following = false; // lost: the eyes go back to the center
        err_x = err_y = 0;
    }
    return moved;
}

// Search for a face: no face for SEARCH_AFTER_US, so look at a random spot
// 33 to 60 degrees away, turn there slowly, stay SEARCH_STAY_US, then look
// somewhere else. Returns the move time, or 0.
static int search_step(int64_t now)
{
    const int64_t quiet_since = last_seen > since ? last_seen : since;
    if (following || page_moved(now) || !tr_web_servos() || now - quiet_since < SEARCH_AFTER_US ||
        now < next_look || !power_for_moves(now) || tr_web_head_power() != ESP_OK) {
        return 0;
    }
    tr_servo_state_t st;
    float yaw = follow_yaw, pitch = follow_pitch;
    if (tr_servo_read(TR_SERVO_YAW, &st) == ESP_OK) {
        yaw = st.degrees;
    }
    if (tr_servo_read(TR_SERVO_PITCH, &st) == ESP_OK) {
        pitch = st.degrees;
    }
    float to_yaw, to_pitch;
    for (int tries = 0;; tries++) { // a spot far enough to show a new part of the room
        to_yaw = (float)((int)(esp_random() % (2 * SEARCH_YAW + 1)) - SEARCH_YAW);
        to_pitch =
            (float)(SEARCH_PITCH_LOW + esp_random() % (SEARCH_PITCH_HIGH - SEARCH_PITCH_LOW + 1));
        const float dyaw = fabsf(to_yaw - yaw), dpitch = fabsf(to_pitch - pitch);
        const bool new_place = dyaw >= SEARCH_MIN_YAW || dpitch >= SEARCH_MIN_PITCH;
        if ((new_place && dyaw <= SEARCH_MAX_STEP && dpitch <= SEARCH_MAX_STEP) || tries >= 20) {
            break;
        }
    }
    // Both axes arrive together, at about 20 degrees per second for the longer one.
    const float far = fmaxf(fabsf(to_yaw - yaw), fabsf(to_pitch - pitch));
    const int ms = (int)(far * SEARCH_MS_PER_DEG) < 400 ? 400 : (int)(far * SEARCH_MS_PER_DEG);
    tr_servo_move(TR_SERVO_YAW, to_yaw, ms);
    tr_servo_move(TR_SERVO_PITCH, to_pitch, ms);
    follow_yaw = to_yaw;
    follow_pitch = to_pitch;
    next_look = now + ms * 1000LL + SEARCH_STAY_US;
    return ms;
}

void track_step(int64_t now, bool quiet, bool move)
{
    if (!finder || failed || quiet || now < hold_until) {
        return;
    }
    if (xSemaphoreTake(head_lock, 0) != pdTRUE) {
        return; // track_quiet() is at work
    }
    // track_quiet() may have run since the check above: look again, now
    // with the lock. Without this the camera started just as sleep cut its
    // power.
    if (esp_timer_get_time() < hold_until || tr_sleep_asleep()) {
        xSemaphoreGive(head_lock);
        return;
    }
    if (!want) { // start, or start again after a talk
        want = true;
        following = false;
        since = now;
        next_look = 0;
        xTaskNotifyGive(finder);
        // Home at boot: straight ahead, pitch at its low safe end.
        if (!homed && tr_web_servos() && power_for_moves(now) && tr_web_head_power() == ESP_OK) {
            ESP_LOGI(TAG, "head: home (boot)");
            tr_servo_move(TR_SERVO_YAW, HOME_YAW, HOME_MS);
            tr_servo_move(TR_SERVO_PITCH, HOME_PITCH, HOME_MS);
            follow_yaw = HOME_YAW;
            follow_pitch = HOME_PITCH;
            release_at = now + (HOME_MS + 300) * 1000;
        }
        homed = true;
    }
    if (loaded) {
        if (follow_step(now, move)) {
            release_at = now + RELEASE_US; // torque off when it stops moving
        }
        const int look_ms = move ? search_step(now) : 0;
        if (look_ms) {
            release_at = now + (look_ms + 300) * 1000LL; // still once it gets there
        }
    }
    // The head is still. While it follows a face, only the torque goes off,
    // so the next move starts at once. With no face the servo power goes off
    // too (about 30 mA less).
    if (release_at && now >= release_at) {
        release_at = 0;
        if (following) {
            tr_servo_torque(TR_SERVO_YAW, false);
            tr_servo_torque(TR_SERVO_PITCH, false);
        } else {
            tr_web_head_off();
        }
    }
    xSemaphoreGive(head_lock);
}

// A box of 2-pixel lines on an RGB565 frame (high byte first).
static void draw_box(camera_fb_t *fb, const face_box_t *b, uint16_t color)
{
    const int w = fb->width, h = fb->height;
    for (int y = b->y0; y <= b->y1; y++) {
        for (int x = b->x0; x <= b->x1; x++) {
            const bool edge = x - b->x0 < 2 || b->x1 - x < 2 || y - b->y0 < 2 || b->y1 - y < 2;
            if (edge && x >= 0 && x < w && y >= 0 && y < h) {
                uint8_t *p = fb->buf + ((size_t)y * w + x) * 2;
                p[0] = color >> 8;
                p[1] = color & 0xFF;
            }
        }
    }
}

// A red box with an X at the top right of the frame, so it is clear how to
// leave. (As in 08-face.)
static void draw_exit_mark(camera_fb_t *fb)
{
    const uint16_t red = TR_LCD_RGB(200, 40, 40), white = TR_LCD_WHITE;
    for (int y = 0; y < 32 && y < (int)fb->height; y++) {
        for (int x = fb->width - 40; x < (int)fb->width; x++) {
            const int dx = x - (fb->width - 40), d1 = dx - y - 4, d2 = dx + y - 36;
            const bool cross =
                dx > 8 && dx < 32 && y > 4 && y < 28 && (abs(d1) <= 1 || abs(d2) <= 1);
            const uint16_t c = cross ? white : red;
            uint8_t *p = fb->buf + ((size_t)y * fb->width + x) * 2;
            p[0] = c >> 8;
            p[1] = c & 0xFF;
        }
    }
}

void track_show(bool on)
{
    if (!finder || on == show) {
        return;
    }
    show = on;
    if (on) {
        xTaskNotifyGive(finder); // no break before the first frame
    } else {
        found_frame_t f;
        if (xQueueReceive(to_show, &f, 0) == pdTRUE) {
            esp_camera_fb_return(f.fb);
        }
    }
}

void track_frame(void)
{
    if (!finder) {
        return;
    }
    taskENTER_CRITICAL(&show_lock);
    const bool no = stopping;
    showing = !no;
    taskEXIT_CRITICAL(&show_lock);
    if (no) {
        return;
    }
    found_frame_t f;
    if (xQueueReceive(to_show, &f, 1) == pdTRUE) {
        for (int i = 0; i < f.n; i++) { // green: the biggest, the one the head follows
            draw_box(f.fb, &f.boxes[i], i == 0 ? TR_SCREEN_GREEN : TR_SCREEN_YELLOW);
        }
        // The center and the dead zone, to check that the head centers the face.
        const int cx = f.fb->width / 2, cy = f.fb->height / 2;
        const int dx = (int)(FOLLOW_DEAD * cx), dy = (int)(FOLLOW_DEAD * cy);
        const face_box_t dead = {cx - dx, cy - dy, cx + dx, cy + dy, 0};
        const face_box_t cross_h = {cx - 8, cy, cx + 8, cy + 1, 0},
                         cross_v = {cx, cy - 8, cx + 1, cy + 8, 0};
        draw_box(f.fb, &dead, CENTER_FG);
        draw_box(f.fb, &cross_h, CENTER_FG);
        draw_box(f.fb, &cross_v, CENTER_FG);
        draw_exit_mark(f.fb);
        tr_lcd_blit(0, 0, f.fb->width, f.fb->height, f.fb->buf);
        esp_camera_fb_return(f.fb);
        // How many frames reach the screen, over 2 s, for GET /face.
        static int shows;
        static int64_t window;
        const int64_t now = esp_timer_get_time();
        shows++;
        if (now - window >= 2000000) {
            show_fps10 = window ? (int)(shows * 10000000LL / (now - window)) : 0;
            shows = 0;
            window = now;
        }
    }
    taskENTER_CRITICAL(&show_lock);
    showing = false;
    taskEXIT_CRITICAL(&show_lock);
}

void track_eyes(float *x, float *y)
{
    *x = err_x;
    *y = err_y;
}

int track_status(char *text, size_t size)
{
    if (!finder || failed) {
        return snprintf(text, size, "track: off (no camera, or no memory)\n");
    }
    if (!loaded) {
        return snprintf(text, size, "track: %s\n",
                        want ? "starting" : "stopped, the camera is off");
    }
    if (show) {
        return snprintf(
            text, size,
            "track: %d face(s), find %d ms, %d.%d finds/s, %d.%d frames/s on the screen\n", faces,
            find_ms, fps10 / 10, fps10 % 10, show_fps10 / 10, show_fps10 % 10);
    }
    return snprintf(text, size, "track: %d face(s), find %d ms, %d.%d finds/s\n", faces, find_ms,
                    fps10 / 10, fps10 % 10);
}
