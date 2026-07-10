#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>

int main(void) {
    printf("Hello from drone!\\n");
    fflush(stdout);

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        printf("socket failed\\n");
        return 1;
    }
    printf("socket OK, fd=%d\\n", fd);
    close(fd);
    return 0;
}
