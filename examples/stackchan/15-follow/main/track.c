// track: see track.h.
//
// One task, the finder, on core 1. It owns the camera and the face model:
// it starts them when the face task wants to follow, and stops and frees
// them for a talk or a recording. The echo canceller needs the same core,
// and a talk needs the internal RAM, so only one of the two runs at a time.
//
// The face task calls track_step() every frame. It reads what the finder
// found and moves the head: toward the face, or to a new spot when there
// has been no face for a while. The rules and numbers are 11-track's.
#include "track.h"

#include <math.h>
#include <stdio.h>
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "face.h"
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "tr_board.h"
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

static TaskHandle_t finder;
static SemaphoreHandle_t head_lock; // the servos: track_step() or track_quiet()
static volatile bool want;          // the finder should run
static volatile bool loaded;        // the camera and the model are up
static volatile bool failed;        // no camera or no memory: don't try again
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
                face_stop();
                tr_cam_stop();
                faces = 0;
                loaded = false;
                log_memory("camera and model off", t0);
            }
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            continue;
        }
        if (!loaded) {
            const int64_t t0 = esp_timer_get_time();
            if (tr_cam_start() != ESP_OK) {
                ESP_LOGW(TAG, "no camera: the head does not follow");
                failed = true;
                want = false;
                continue;
            }
            if (face_start() != ESP_OK) {
                ESP_LOGW(TAG, "no memory for the face model: the head does not follow");
                tr_cam_stop();
                failed = true;
                want = false;
                continue;
            }
            loaded = true;
            finds = ms_sum = 0;
            window = esp_timer_get_time();
            face_at = -1;
            log_memory("camera and model on", t0);
        }
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            continue; // the camera timed out; try again
        }
        face_box_t boxes[FACE_MAX];
        const int64_t t0 = esp_timer_get_time();
        const int n = face_find(fb->buf, fb->width, fb->height, boxes, FACE_MAX);
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
        esp_camera_fb_return(fb);
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
        // No face for a while: a break. A notify ends it at once.
        if (face_at < 0 || now - face_at > LOST_US) {
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(SLOW_FIND_MS));
        }
    }
}

esp_err_t track_init(void)
{
    head_lock = xSemaphoreCreateMutex();
    if (!head_lock ||
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
    for (int i = 0; i < 300 && loaded; i++) { // the finder ends its frame first
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (loaded) {
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

// Turn toward the face. True if the head moved.
static bool follow_step(int64_t now)
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
            if (!page_moved(now) && tr_web_servos() && power_for_moves(now) &&
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

void track_step(int64_t now, bool quiet)
{
    if (!finder || failed || quiet || now < hold_until) {
        return;
    }
    if (xSemaphoreTake(head_lock, 0) != pdTRUE) {
        return; // track_quiet() is at work
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
        if (follow_step(now)) {
            release_at = now + RELEASE_US; // torque off when it stops moving
        }
        const int look_ms = search_step(now);
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
                        want ? "starting" : "stopped (talk or recording)");
    }
    return snprintf(text, size, "track: %d face(s), find %d ms, %d.%d finds/s\n", faces, find_ms,
                    fps10 / 10, fps10 % 10);
}
