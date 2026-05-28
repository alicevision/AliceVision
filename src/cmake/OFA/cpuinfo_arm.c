#include <stdio.h>
#include <stdint.h>
#include <setjmp.h>
#include <signal.h>
#include <stdlib.h>

sigjmp_buf go_here;

void sigill_handler(int signum)
{
    (void)signum;
    siglongjmp(go_here, 1);
}

int main(void)
{
    struct sigaction sa;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sa.sa_handler = sigill_handler;
    if (sigaction(SIGILL, &sa, NULL) < 0)
    {
        perror("sigaction");
        exit(2);
    }

    do
    {
        if (sigsetjmp(go_here, 1))
        {
            exit(-1);
        }
        else
        {
            unsigned long ret;
            asm("mrs %0, MIDR_EL1" : "=r"(ret));

            printf("%s 0x%02lX\n", "[OptimizeForArchitecture] CPU implementer :", (ret >> 24) & 0xFF);
            printf("%s 0x%01lX\n", "[OptimizeForArchitecture] CPU architecture:", (ret >> 16) & 0xF);
            printf("%s 0x%01lX\n", "[OptimizeForArchitecture] CPU variant     :", (ret >> 20) & 0xF);
            printf("%s 0x%03lX\n", "[OptimizeForArchitecture] CPU part        :", (ret >> 4) & 0xFFF);
            printf("%s %ld\n", "[OptimizeForArchitecture] CPU revision    :", ret & 0xF);
        }
    } while (0);

    return 0;
}
