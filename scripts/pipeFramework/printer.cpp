#include <stdio.h>
#include <stdlib.h>
#include <sndfile.h>
#include <unistd.h>

int main()
    {
    SNDFILE *sf;
    SF_INFO info;
    int num_channels;
    int num, num_items;
    float *buf;
    int f,sr,c;
    int i,j;
    //FILE *out;
    SNDFILE *out;
    
    /* Open the RAW file. */
    info.format = SF_FORMAT_WAV;
    sf = sf_open_fd(0, SFM_READ, &info, true);
    if (sf == NULL)
        {
        printf("Failed to open the file.\n");
        exit(-1);
        }
    /* Print some of the info, and figure out how much data to read. */
    f = info.frames;
    sr = info.samplerate;
    c = info.channels;
    printf("frames=%d\n",f);
    printf("samplerate=%d\n",sr);
    printf("channels=%d\n",c);
    num_items = 100; //f*c;
    printf("num_items=%d\n",num_items);
    /* Allocate space for the data to be read, then read it. */
    buf = (float *) malloc(num_items*sizeof(float));
    // num = sf_read_int(sf,buf,num_items);
    // printf("%i", num_items);
    // sf_close(sf);
    printf("Read %d items\n",num);
    /* Write the data to filedata.out. */
    out = sf_open("filedata.out",SFM_WRITE, &info);
    while ((num = sf_read_float (sf, buf, num_items)) > 0) {
        printf("%f  %f\r", buf[0], buf[1]);
        sf_write_float (out, buf, num) ;
    }
    sf_close(sf);
    sf_close(out);
    // out = fopen("filedata.out","w");
    // for (i = 0; i < num; i += c)
    //     {
    //     for (j = 0; j < c; ++j)
    //         fprintf(out,"%d ",buf[i+j]);
    //     fprintf(out,"\n");
    //     }
    // fclose(out);
    return 0;
    }