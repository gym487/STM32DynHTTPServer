// ------------------  ASCII字模的数据表 ------------------------ //
// 码表从0x20~0x7e                                                //
// 字库: F:\lcd汉字取模软件\Asc8X16E.dat 横向取模左高位           //
// -------------------------------------------------------------- //
const uint8_t charss[]={
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,  // - -
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

	0x00,0x00,0x00,0x00,0x38,0x00,0xFC,0x0D,  // -!-
	0xFC,0x0D,0x38,0x00,0x00,0x00,0x00,0x00,

	0x00,0x00,0x0E,0x00,0x1E,0x00,0x00,0x00,  // -"-
	0x00,0x00,0x1E,0x00,0x0E,0x00,0x00,0x00,

	0x20,0x02,0xF8,0x0F,0xF8,0x0F,0x20,0x02,  // -#-
	0xF8,0x0F,0xF8,0x0F,0x20,0x02,0x00,0x00,

	0x38,0x03,0x7C,0x06,0x44,0x04,0x47,0x1C,  // -$-
	0x47,0x1C,0xCC,0x07,0x98,0x03,0x00,0x00,

	0x30,0x0C,0x30,0x06,0x00,0x03,0x80,0x01,  // -%-
	0xC0,0x00,0x60,0x0C,0x30,0x0C,0x00,0x00,

	0x80,0x07,0xD8,0x0F,0x7C,0x08,0xE4,0x08,  // -&-
	0xBC,0x07,0xD8,0x0F,0x40,0x08,0x00,0x00,

	0x00,0x00,0x10,0x00,0x1E,0x00,0x0E,0x00,  // -'-
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

	0x00,0x00,0x00,0x00,0xF0,0x03,0xF8,0x07,  // -(-
	0x0C,0x0C,0x04,0x08,0x00,0x00,0x00,0x00,

	0x00,0x00,0x00,0x00,0x04,0x08,0x0C,0x0C,  // -)-
	0xF8,0x07,0xF0,0x03,0x00,0x00,0x00,0x00,

	0x80,0x00,0xA0,0x02,0xE0,0x03,0xC0,0x01,  // -*-
	0xC0,0x01,0xE0,0x03,0xA0,0x02,0x80,0x00,

	0x00,0x00,0x80,0x00,0x80,0x00,0xE0,0x03,  // -+-
	0xE0,0x03,0x80,0x00,0x80,0x00,0x00,0x00,

	0x00,0x00,0x00,0x00,0x00,0x10,0x00,0x1E,  // -,-
	0x00,0x0E,0x00,0x00,0x00,0x00,0x00,0x00,

	0x80,0x00,0x80,0x00,0x80,0x00,0x80,0x00,  // ---
	0x80,0x00,0x80,0x00,0x80,0x00,0x00,0x00,

	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0C,  // -.-
	0x00,0x0C,0x00,0x00,0x00,0x00,0x00,0x00,

	0x00,0x0C,0x00,0x06,0x00,0x03,0x80,0x01,  // -/-
	0xC0,0x00,0x60,0x00,0x30,0x00,0x00,0x00,

	0xF8,0x07,0xFC,0x0F,0x04,0x09,0xC4,0x08,  // -0-
	0x24,0x08,0xFC,0x0F,0xF8,0x07,0x00,0x00,

	0x00,0x00,0x10,0x08,0x18,0x08,0xFC,0x0F,  // -1-
	0xFC,0x0F,0x00,0x08,0x00,0x08,0x00,0x00,

	0x08,0x0E,0x0C,0x0F,0x84,0x09,0xC4,0x08,  // -2-
	0x64,0x08,0x3C,0x0C,0x18,0x0C,0x00,0x00,

	0x08,0x04,0x0C,0x0C,0x44,0x08,0x44,0x08,  // -3-
	0x44,0x08,0xFC,0x0F,0xB8,0x07,0x00,0x00,

	0xC0,0x00,0xE0,0x00,0xB0,0x00,0x98,0x08,  // -4-
	0xFC,0x0F,0xFC,0x0F,0x80,0x08,0x00,0x00,

	0x7C,0x04,0x7C,0x0C,0x44,0x08,0x44,0x08,  // -5-
	0xC4,0x08,0xC4,0x0F,0x84,0x07,0x00,0x00,

	0xF0,0x07,0xF8,0x0F,0x4C,0x08,0x44,0x08,  // -6-
	0x44,0x08,0xC0,0x0F,0x80,0x07,0x00,0x00,

	0x0C,0x00,0x0C,0x00,0x04,0x0F,0x84,0x0F,  // -7-
	0xC4,0x00,0x7C,0x00,0x3C,0x00,0x00,0x00,

	0xB8,0x07,0xFC,0x0F,0x44,0x08,0x44,0x08,  // -8-
	0x44,0x08,0xFC,0x0F,0xB8,0x07,0x00,0x00,

	0x38,0x00,0x7C,0x08,0x44,0x08,0x44,0x08,  // -9-
	0x44,0x0C,0xFC,0x07,0xF8,0x03,0x00,0x00,

	0x00,0x00,0x00,0x00,0x00,0x00,0x30,0x06,  // -:-
	0x30,0x06,0x00,0x00,0x00,0x00,0x00,0x00,

	0x00,0x00,0x00,0x00,0x00,0x08,0x30,0x0E,  // -;-
	0x30,0x06,0x00,0x00,0x00,0x00,0x00,0x00,

	0x00,0x00,0x80,0x00,0xC0,0x01,0x60,0x03,  // -<-
	0x30,0x06,0x18,0x0C,0x08,0x08,0x00,0x00,

	0x40,0x02,0x40,0x02,0x40,0x02,0x40,0x02,  // -=-
	0x40,0x02,0x40,0x02,0x40,0x02,0x00,0x00,

	0x00,0x00,0x08,0x08,0x18,0x0C,0x30,0x06,  // ->-
	0x60,0x03,0xC0,0x01,0x80,0x00,0x00,0x00,

	0x18,0x00,0x1C,0x00,0x04,0x00,0xC4,0x0D,  // -?-
	0xE4,0x0D,0x3C,0x00,0x18,0x00,0x00,0x00,

	0xF0,0x07,0xF8,0x0F,0x08,0x08,0xC8,0x0B,  // -@-
	0xC8,0x0B,0xF8,0x0B,0xF0,0x01,0x00,0x00,

	0xE0,0x0F,0xF0,0x0F,0x98,0x00,0x8C,0x00,  // -A-
	0x98,0x00,0xF0,0x0F,0xE0,0x0F,0x00,0x00,

	0x04,0x08,0xFC,0x0F,0xFC,0x0F,0x44,0x08,  // -B-
	0x44,0x08,0xFC,0x0F,0xB8,0x07,0x00,0x00,

	0xF0,0x03,0xF8,0x07,0x0C,0x0C,0x04,0x08,  // -C-
	0x04,0x08,0x0C,0x0C,0x18,0x06,0x00,0x00,

	0x04,0x08,0xFC,0x0F,0xFC,0x0F,0x04,0x08,  // -D-
	0x0C,0x0C,0xF8,0x07,0xF0,0x03,0x00,0x00,

	0x04,0x08,0xFC,0x0F,0xFC,0x0F,0x44,0x08,  // -E-
	0xE4,0x08,0x0C,0x0C,0x1C,0x0E,0x00,0x00,

	0x04,0x08,0xFC,0x0F,0xFC,0x0F,0x44,0x08,  // -F-
	0xE4,0x00,0x0C,0x00,0x1C,0x00,0x00,0x00,

	0xF0,0x03,0xF8,0x07,0x0C,0x0C,0x84,0x08,  // -G-
	0x84,0x08,0x8C,0x07,0x98,0x0F,0x00,0x00,

	0xFC,0x0F,0xFC,0x0F,0x40,0x00,0x40,0x00,  // -H-
	0x40,0x00,0xFC,0x0F,0xFC,0x0F,0x00,0x00,

	0x00,0x00,0x00,0x00,0x04,0x08,0xFC,0x0F,  // -I-
	0xFC,0x0F,0x04,0x08,0x00,0x00,0x00,0x00,

	0x00,0x07,0x00,0x0F,0x00,0x08,0x04,0x08,  // -J-
	0xFC,0x0F,0xFC,0x07,0x04,0x00,0x00,0x00,

	0x04,0x08,0xFC,0x0F,0xFC,0x0F,0xC0,0x00,  // -K-
	0xF0,0x01,0x3C,0x0F,0x0C,0x0E,0x00,0x00,

	0x04,0x08,0xFC,0x0F,0xFC,0x0F,0x04,0x08,  // -L-
	0x00,0x08,0x00,0x0C,0x00,0x0E,0x00,0x00,

	0xFC,0x0F,0xFC,0x0F,0x38,0x00,0x70,0x00,  // -M-
	0x38,0x00,0xFC,0x0F,0xFC,0x0F,0x00,0x00,

	0xFC,0x0F,0xFC,0x0F,0x38,0x00,0x70,0x00,  // -N-
	0xE0,0x00,0xFC,0x0F,0xFC,0x0F,0x00,0x00,

	0xF0,0x03,0xF8,0x07,0x0C,0x0C,0x04,0x08,  // -O-
	0x0C,0x0C,0xF8,0x07,0xF0,0x03,0x00,0x00,

	0x04,0x08,0xFC,0x0F,0xFC,0x0F,0x44,0x08,  // -P-
	0x44,0x00,0x7C,0x00,0x38,0x00,0x00,0x00,

	0xF8,0x07,0xFC,0x0F,0x04,0x08,0x04,0x0E,  // -Q-
	0x04,0x3C,0xFC,0x3F,0xF8,0x27,0x00,0x00,

	0x04,0x08,0xFC,0x0F,0xFC,0x0F,0x44,0x00,  // -R-
	0xC4,0x00,0xFC,0x0F,0x38,0x0F,0x00,0x00,

	0x18,0x06,0x3C,0x0E,0x64,0x08,0x44,0x08,  // -S-
	0xC4,0x08,0x9C,0x0F,0x18,0x07,0x00,0x00,

	0x00,0x00,0x1C,0x00,0x0C,0x08,0xFC,0x0F,  // -T-
	0xFC,0x0F,0x0C,0x08,0x1C,0x00,0x00,0x00,

	0xFC,0x07,0xFC,0x0F,0x00,0x08,0x00,0x08,  // -U-
	0x00,0x08,0xFC,0x0F,0xFC,0x07,0x00,0x00,

	0xFC,0x01,0xFC,0x03,0x00,0x06,0x00,0x0C,  // -V-
	0x00,0x06,0xFC,0x03,0xFC,0x01,0x00,0x00,

	0xFC,0x03,0xFC,0x0F,0x00,0x0E,0x80,0x03,  // -W-
	0x00,0x0E,0xFC,0x0F,0xFC,0x03,0x00,0x00,

	0x0C,0x0C,0x3C,0x0F,0xF0,0x03,0xC0,0x00,  // -X-
	0xF0,0x03,0x3C,0x0F,0x0C,0x0C,0x00,0x00,

	0x00,0x00,0x3C,0x00,0x7C,0x08,0xC0,0x0F,  // -Y-
	0xC0,0x0F,0x7C,0x08,0x3C,0x00,0x00,0x00,

	0x1C,0x0E,0x0C,0x0F,0x84,0x09,0xC4,0x08,  // -Z-
	0x64,0x08,0x3C,0x0C,0x1C,0x0E,0x00,0x00,

	0x00,0x00,0x00,0x00,0xFC,0x0F,0xFC,0x0F,  // -[-
	0x04,0x08,0x04,0x08,0x00,0x00,0x00,0x00,

	0x38,0x00,0x70,0x00,0xE0,0x00,0xC0,0x01,  // -\-
	0x80,0x03,0x00,0x07,0x00,0x0E,0x00,0x00,

	0x00,0x00,0x00,0x00,0x04,0x08,0x04,0x08,  // -]-
	0xFC,0x0F,0xFC,0x0F,0x00,0x00,0x00,0x00,

	0x08,0x00,0x0C,0x00,0x06,0x00,0x03,0x00,  // -^-
	0x06,0x00,0x0C,0x00,0x08,0x00,0x00,0x00,

	0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x20,  // -_-
	0x00,0x20,0x00,0x20,0x00,0x20,0x00,0x20,

	0x00,0x00,0x00,0x00,0x03,0x00,0x07,0x00,  // -`-
	0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,

	0x00,0x07,0xA0,0x0F,0xA0,0x08,0xA0,0x08,  // -a-
	0xE0,0x07,0xC0,0x0F,0x00,0x08,0x00,0x00,

	0x04,0x08,0xFC,0x0F,0xFC,0x07,0x20,0x08,  // -b-
	0x60,0x08,0xC0,0x0F,0x80,0x07,0x00,0x00,

	0xC0,0x07,0xE0,0x0F,0x20,0x08,0x20,0x08,  // -c-
	0x20,0x08,0x60,0x0C,0x40,0x04,0x00,0x00,

	0x80,0x07,0xC0,0x0F,0x60,0x08,0x24,0x08,  // -d-
	0xFC,0x07,0xFC,0x0F,0x00,0x08,0x00,0x00,

	0xC0,0x07,0xE0,0x0F,0xA0,0x08,0xA0,0x08,  // -e-
	0xA0,0x08,0xE0,0x0C,0xC0,0x04,0x00,0x00,

	0x40,0x08,0xF8,0x0F,0xFC,0x0F,0x44,0x08,  // -f-
	0x0C,0x00,0x18,0x00,0x00,0x00,0x00,0x00,

	0xC0,0x27,0xE0,0x6F,0x20,0x48,0x20,0x48,  // -g-
	0xC0,0x7F,0xE0,0x3F,0x20,0x00,0x00,0x00,

	0x04,0x08,0xFC,0x0F,0xFC,0x0F,0x40,0x00,  // -h-
	0x20,0x00,0xE0,0x0F,0xC0,0x0F,0x00,0x00,

	0x00,0x00,0x00,0x00,0x20,0x08,0xEC,0x0F,  // -i-
	0xEC,0x0F,0x00,0x08,0x00,0x00,0x00,0x00,

	0x00,0x00,0x00,0x30,0x00,0x70,0x00,0x40,  // -j-
	0x20,0x40,0xEC,0x7F,0xEC,0x3F,0x00,0x00,

	0x04,0x08,0xFC,0x0F,0xFC,0x0F,0x80,0x01,  // -k-
	0xC0,0x03,0x60,0x0E,0x20,0x0C,0x00,0x00,

	0x00,0x00,0x00,0x00,0x04,0x08,0xFC,0x0F,  // -l-
	0xFC,0x0F,0x00,0x08,0x00,0x00,0x00,0x00,

	0xE0,0x0F,0xE0,0x0F,0x60,0x00,0xC0,0x0F,  // -m-
	0x60,0x00,0xE0,0x0F,0xC0,0x0F,0x00,0x00,

	0x20,0x00,0xE0,0x0F,0xC0,0x0F,0x20,0x00,  // -n-
	0x20,0x00,0xE0,0x0F,0xC0,0x0F,0x00,0x00,

	0xC0,0x07,0xE0,0x0F,0x20,0x08,0x20,0x08,  // -o-
	0x20,0x08,0xE0,0x0F,0xC0,0x07,0x00,0x00,

	0x20,0x40,0xE0,0x7F,0xC0,0x7F,0x20,0x48,  // -p-
	0x20,0x08,0xE0,0x0F,0xC0,0x07,0x00,0x00,

	0xC0,0x07,0xE0,0x0F,0x20,0x08,0x20,0x48,  // -q-
	0xC0,0x7F,0xE0,0x7F,0x20,0x40,0x00,0x00,

	0x20,0x08,0xE0,0x0F,0xC0,0x0F,0x60,0x08,  // -r-
	0x20,0x00,0x60,0x00,0xC0,0x00,0x00,0x00,

	0x40,0x04,0xE0,0x0C,0xA0,0x09,0x20,0x09,  // -s-
	0x20,0x0B,0x60,0x0E,0x40,0x04,0x00,0x00,

	0x20,0x00,0x20,0x00,0xF8,0x07,0xFC,0x0F,  // -t-
	0x20,0x08,0x20,0x0C,0x00,0x04,0x00,0x00,

	0xE0,0x07,0xE0,0x0F,0x00,0x08,0x00,0x08,  // -u-
	0xE0,0x07,0xE0,0x0F,0x00,0x08,0x00,0x00,

	0x00,0x00,0xE0,0x03,0xE0,0x07,0x00,0x0C,  // -v-
	0x00,0x0C,0xE0,0x07,0xE0,0x03,0x00,0x00,

	0xE0,0x07,0xE0,0x0F,0x00,0x0C,0x00,0x07,  // -w-
	0x00,0x0C,0xE0,0x0F,0xE0,0x07,0x00,0x00,

	0x20,0x08,0x60,0x0C,0xC0,0x07,0x80,0x03,  // -x-
	0xC0,0x07,0x60,0x0C,0x20,0x08,0x00,0x00,

	0xE0,0x47,0xE0,0x4F,0x00,0x48,0x00,0x48,  // -y-
	0x00,0x68,0xE0,0x3F,0xE0,0x1F,0x00,0x00,

	0x60,0x0C,0x60,0x0E,0x20,0x0B,0xA0,0x09,  // -z-
	0xE0,0x08,0x60,0x0C,0x20,0x0C,0x00,0x00,

	0x00,0x00,0x40,0x00,0x40,0x00,0xF8,0x07,  // -{-
	0xBC,0x0F,0x04,0x08,0x04,0x08,0x00,0x00,

	0x00,0x00,0x00,0x00,0x00,0x00,0xBC,0x0F,  // -|-
	0xBC,0x0F,0x00,0x00,0x00,0x00,0x00,0x00,

	0x00,0x00,0x04,0x08,0x04,0x08,0xBC,0x0F,  // -}-
	0xF8,0x07,0x40,0x00,0x40,0x00,0x00,0x00,

	0x08,0x00,0x0C,0x00,0x04,0x00,0x0C,0x00,  // -~-
	0x08,0x00,0x0C,0x00,0x04,0x00,0x00,0x00,

	0x80,0x07,0xC0,0x07,0x60,0x04,0x30,0x04,  // --
	0x60,0x04,0xC0,0x07,0x80,0x07,0x00,0x00,
};



void charcpy(int line,int offset,uint8_t dd[8][128],unsigned char a,uint8_t mask){
  int i;
  if(a>=32&&a<127){
    for(i=0;i<8;i++){
      dd[line*2][i+offset*8]=mask^charss[(a-32)*16+i*2];
      dd[line*2+1][i+offset*8]=mask^charss[(a-32)*16+i*2+1];
    }
  }else{
    for(i=0;i<8;i+=2){
      dd[line*2][i+offset*8]=0xff;
      dd[line*2+1][i+offset*8]=0xff;
    }
  }
}