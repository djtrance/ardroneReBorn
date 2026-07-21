#ifndef MDNS_H
#define MDNS_H

int mdns_init(void);
void *mdns_thread(void *arg);
void mdns_stop(void);

#endif
