#ifndef RC_TEST_APP_H
#define RC_TEST_APP_H

void rc_test_init(void); /* once, after the CubeMX MX_*_Init() calls */
void rc_test_loop(void); /* forever, inside main()'s while (1) */

#endif
