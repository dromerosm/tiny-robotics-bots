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
    int n = 0;
    for (const auto &r : detect->run(img)) { // strongest first
        if (n >= max || r.box.size() < 4) {
            break;
        }
        boxes[n++] = {r.box[0], r.box[1], r.box[2], r.box[3], r.score};
    }
    return n;
}
