// face: see face.h.
#include "face.h"

#include <new>
#include "human_face_detect.hpp"

static HumanFaceDetect *detect;

esp_err_t face_start(void)
{
    if (!detect) {
        // Load now (not lazily) so the first frame is not slow.
        detect = new (std::nothrow) HumanFaceDetect(HumanFaceDetect::MSRMNP_S8_V1, false);
    }
    return detect ? ESP_OK : ESP_ERR_NO_MEM;
}

void face_stop(void)
{
    delete detect;
    detect = nullptr;
}

int face_find(const uint8_t *rgb565, int w, int h, face_box_t *boxes, int max)
{
    if (!detect) {
        return 0;
    }
    const dl::image::img_t img = {
        .data = (void *)rgb565,
        .width = (uint16_t)w,
        .height = (uint16_t)h,
        .pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565BE,
    };
    // Keep the biggest faces, biggest first: the biggest is usually the
    // closest person, and that is the one the head follows.
    int n = 0;
    for (const auto &r : detect->run(img)) {
        if (r.box.size() < 4) {
            continue;
        }
        const face_box_t b = {r.box[0], r.box[1], r.box[2], r.box[3], r.score};
        const int area = (b.x1 - b.x0) * (b.y1 - b.y0);
        int at = n < max ? n++ : max; // where it goes; max: smaller than all kept
        while (at > 0 &&
               (boxes[at - 1].x1 - boxes[at - 1].x0) * (boxes[at - 1].y1 - boxes[at - 1].y0) <
                   area) {
            if (at < max) {
                boxes[at] = boxes[at - 1];
            }
            at--;
        }
        if (at < max) {
            boxes[at] = b;
        }
    }
    return n;
}
