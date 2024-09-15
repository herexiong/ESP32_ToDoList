#ifndef AUDIO_DLNA_H_
#define AUDIO_DLNA_H_

#define WiFi_SSID "there isnot wifi"
#define WiFi_PWD "1234567890987654321"

char *trans_state;
void dlna_notify(void);

void dlna_init_task(void *param);
void dlna_deinit_task(void *param);

#endif