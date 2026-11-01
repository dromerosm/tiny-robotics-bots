// tr_screen: text screens on the robot, and the status screen that uses them.
//
// A text screen has a title bar with X, 8 rows of text and two buttons at the
// bottom. The status screen shows one section per page: battery (with the
// time left and the sleep state), robot, network (with the update token),
// touch and motion, and the room. It is the same as the cards on the page.
// Call these from one task only (the one that draws).
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "tr_board.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TR_SCREEN_COLS   26 // characters per row
#define TR_SCREEN_ROWS   8  // rows between the two bars
#define TR_SCREEN_BAR_H  28 // the title bar and the buttons
#define TR_SCREEN_FG     TR_LCD_RGB(255, 255, 255)
#define TR_SCREEN_GREEN  TR_LCD_RGB(80, 220, 100)
#define TR_SCREEN_YELLOW TR_LCD_RGB(240, 200, 60)
#define TR_SCREEN_RED    TR_LCD_RGB(240, 80, 80)

// The lines of a text screen (up to 48). clear empties them, row adds one,
// cut at TR_SCREEN_COLS characters. count says how many there are.
void tr_screen_clear(void);
void tr_screen_row(uint16_t color, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
int tr_screen_count(void);

// Draw the whole screen: title, the two button labels, and the lines from top.
void tr_screen_draw(const char *title, const char *left, const char *right, int top);
// Draw only the rows again, from top (after new lines).
void tr_screen_rows(int top);
// Move top by delta rows and keep it in range. Returns the new top.
int tr_screen_scroll(int top, int delta);

// Where a tap is: on the title bar (closes the screen) or on a button
// (-1 left, 1 right, 0 none).
bool tr_screen_on_bar(int y);
int tr_screen_button(int x, int y);

// The status screen. open draws the first section, update draws new numbers
// (call it once a second), tap handles PREV and NEXT.
void tr_status_open(void);
void tr_status_update(void);
void tr_status_tap(int x, int y);

#ifdef __cplusplus
}
#endif
