#ifndef ESP_CHATTER_H
#define ESP_CHATTER_H

#ifdef __cplusplus
extern "C" {
#endif

void rosserial_setup();

void rosserial_publish(int enc_msg);

#ifdef __cplusplus
}
#endif

#endif /* ESP_CHATTER_H */
