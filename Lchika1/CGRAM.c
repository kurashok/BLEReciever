/*
 * CGRAM.c
 *
 * Created: 2021/07/07 10:26:16
 *  Author: kuras
 */ 
const char cgramdata[8][8] =
{
	{// 0
		0b00001110,
		0b00011111,
		0b00010001,
		0b00010001,
		0b00010001,
		0b00010001,
		0b00010001,
		0b00011111
	},
	{// 1
		0b00001110,
		0b00011111,
		0b00010001,
		0b00010001,
		0b00010001,
		0b00010001,
		0b00011111,
		0b00011111
	},
	{// 2
		0b00001110,
		0b00011111,
		0b00010001,
		0b00010001,
		0b00010001,
		0b00011111,
		0b00011111,
		0b00011111
	},
	{// 3
		0b00001110,
		0b00011111,
		0b00010001,
		0b00010001,
		0b00011111,
		0b00011111,
		0b00011111,
		0b00011111
	},
	{// 4
		0b00001110,
		0b00011111,
		0b00010001,
		0b00011111,
		0b00011111,
		0b00011111,
		0b00011111,
		0b00011111
	},
	{// 5
		0b00001110,
		0b00011111,
		0b00011111,
		0b00011111,
		0b00011111,
		0b00011111,
		0b00011111,
		0b00011111
	},
	{// 6
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000100,
		0b00001010,
		0b00000100,
		0b00000000,
		0b00000000
	},
	{// 7
		0b00000010,
		0b00000100,
		0b00001000,
		0b00011110,
		0b00000100,
		0b00001000,
		0b00010000,
		0b00000000
	}
};
/*
const char largechar[8][8] =
{
	{// 0
		0b00011111,
		0b00011111,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000000
	},
	{// 1
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00011111,
		0b00011111
	},
	{// 2
		0b00011111,
		0b00011111,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00011111,
		0b00011111
	},
	{// 3
		0b00000011,
		0b00000111,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000001,
		0b00000011
	},
	{// 4
		0b00011100,
		0b00011110,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00010000,
		0b00011000
	},
	{// 5
		0b00000011,
		0b00000111,
		0b00000111,
		0b00001111,
		0b00001111,
		0b00001111,
		0b00001111,
		0b00000111
	},
	{// 6
		0b00011100,
		0b00011110,
		0b00011110,
		0b00011110,
		0b00011110,
		0b00011100,
		0b00011100,
		0b00011000
	},
	{// 7
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00001111,
		0b00000111
	}
};
*/
const char largechar[8][8] =
{
	{// 0
		0b00011111,
		0b00011111,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000000
	},
	{// 1
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00011111,
		0b00011111
	},
	{// 2
		0b00011111,
		0b00011111,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00011111,
		0b00011111
	},
	{// 3
		0b00000011,
		0b00000111,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000011,
		0b00000111
	},
	{// 4
		0b00011000,
		0b00011100,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00011000,
		0b00011100
	},
	{// 5
		0b00000111,
		0b00001111,
		0b00001111,
		0b00001111,
		0b00001111,
		0b00001111,
		0b00001111,
		0b00000111
	},
	{// 6
		0b00011100,
		0b00011110,
		0b00011110,
		0b00011110,
		0b00011110,
		0b00011110,
		0b00011110,
		0b00011100
	},
	{// 7
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000000,
		0b00000111,
		0b00000011
	}
};

// １文字はLCDキャラクタ３文字?２行で構成
// largechar[数字(0-9)][LCDの上下][３文字のCGRAM番号]
const char largenum[10][2][3] =
{
	{{5,0,6},{5,1,6}}, // 0
	{{' ',5,' '},{' ',5,' '}}, // 1
	{{3,2,6},{5,1,1}}, // 2
	{{3,2,6},{7,1,6}}, // 3
	{{5,1,6},{' ',' ',6}}, // 4
	{{5,2,4},{7,1,6}}, // 5
	{{5,2,4},{5,1,6}}, // 6
	{{5,0,6},{' ',' ',6}}, // 7
	{{5,2,6},{5,1,6}}, // 8
	{{5,2,6},{7,1,6}}  // 9
};
