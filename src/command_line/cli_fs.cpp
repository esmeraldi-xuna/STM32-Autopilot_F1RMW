#include <mbed.h>
#include "FATFileSystem.h"
#include "SDBlockDevice.h"

extern FATFileSystem file_sys;

void cli_ls(void)
{
    // open folder
    DIR *d = opendir("sd/");
    if (!d)
    {
        printf("error open dir\n");
    }
    printf("SD content:\n");
    // read all filename
    while (true)
    {
        struct dirent *e = readdir(d);
        if (!e)
        {
            break;
        }
        printf("/sd/%s\n", e->d_name);
    }

    // close folder
    int err = closedir(d);
    if (err < 0)
    {
        printf("error close dir\n");
    }

}

void cli_cat(char *s){
    int i;
    strncpy(s,"/sd/",4);// "cat " substituted with "/sd/"
    FILE *f = fopen(s,"r");
    char c;
    while(!feof(f)){
         c = (char)fgetc(f); 
         putchar(c);
    }
    putchar(0x7f);
    fclose(f);
}