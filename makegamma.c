#include <stdlib.h>
#include <stdio.h>
#include <math.h>

int main(int argc,char *argv[]) {
	FILE *gammafile;
	if ((gammafile=fopen("gamma.h","w")) == NULL) {
		fprintf(stderr,"Could not open output file gamma.h\n");
		exit(1);
	}

	fprintf(gammafile,"static unsigned char gamma_table[256] = {\n");

	for(int i=0;i<256;i+=8) {
		for(int j=0;j<8;j++) {
			fprintf(gammafile,"0x%02x",
				(int)(255*pow((i+j)/255.0,0.65)));
			if (j == 7) {
				if (i==(256-8))
					fputc('\n',gammafile);
				else
					fprintf(gammafile,",\n");
			} else {
				fprintf(gammafile,", ");
			}
		}
	}

	fprintf(gammafile,"};\n");
	fclose(gammafile);

	return 0;
}
