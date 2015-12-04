#include <time.h>
#include <control_r.h>
using namespace std;

void sleepcp(int milliseconds) // cross-platform sleep function
{
	clock_t time_end;
	time_end = clock() + milliseconds * CLOCKS_PER_SEC / 1000;
	while (clock() < time_end)
	{
	}
}

float * move_vector(float liigu[3]){//liigu[3] = liikumise vektor {x, y, w} w-nurkkiirendus, 0 kui ei taha pöörata
	//liikumise maatriks, et ei peaks iga kord arvutama
	float liikumine[3][3] = { { 0.57735, -0.33333, 0.33333 }, { -0.57735, -0.33333, 0.33333 }, {

		0, 0.66667, 0.33333 } };
	float f1, f2, f3, x, y, w;
	static float tagastus[3];//jõudude vektor mille pärast tagastame

	x = liigu[0];
	y = liigu[1];
	w = liigu[2];

	//arvutame iga mootori jõu
	f1 = liikumine[0][0] * x + liikumine[0][1] * y + liikumine[0][2] * w;
	f2 = liikumine[1][0] * x + liikumine[1][1] * y + liikumine[1][2] * w;
	f3 = liikumine[2][0] * x + liikumine[2][1] * y + liikumine[2][2] * w;

	tagastus[0] = f1;
	tagastus[1] = f2;
	tagastus[2] = f3;

	return tagastus;
}

int * get_speed(float * joud, int max_speed){
	static int tagastus[3];

	tagastus[0] = ymarda(joud[0] * max_speed);
	tagastus[1] = ymarda(joud[1] * max_speed);
	tagastus[2] = ymarda(joud[2] * max_speed);

	return tagastus;
}

int ymarda(float a){
	if (a > 0){
		int b = (int)(a + 0.5);
		return (int)b;
	}
	else if (a < 0){
		int b = (int)(a - 0.5);
		return (int)b;
	}
	else return 0;
}
