//2 dimensional circular particle simulator
#include <SDL2/SDL.h>
#include <iostream>
#include <cmath>
#include <vector>

#include <random>
#define rui random_device()()	/*random unsigned int*/

using namespace std;

#define ve vector
SDL_Renderer *ren;
SDL_Rect circlerectangle1,circlerectangle2;	//for drawing filled circles
#define drawline(a,s,d,f) SDL_RenderDrawLine(ren,a,s,d,f)
#define pi 4
#define color SDL_Color
setcolor (color c) {SDL_SetRenderDrawColor(ren,c.r,c.g,c.b,255);}	//sets the drawing color for the renderer
#define sx 888 /*screen length x*/
#define sy 888 /*screen length y*/

color colors[11] = {{222,66,66}, {222,222,222}, {0,33,33}, {111,111,111}};	//COLORS
color baccolor=colors[0], &cdc= colors[1], &coc=colors[2], &tic=colors[3]	//background	circle	connection	timer	
;//baccolor: background color	cdc:circle default color 	coc:connection color 	tic:timer color


class circle {public:
	int radius; color cc; float x,y, mass=1, ax=0,ay=0, vx=0,vy=0;	bool fill=false;	//cc: circle color	mass: mass	fill: fill default
	circle(float x, float y, int radius, color cc=cdc) : x(x),y(y),radius(radius),cc(cc)	{}	//cc: circle color

	draw(bool fi) {	//fi: filled
		setcolor(cc);
		for (float a=0; a<=pi*2; a+=.5)	drawline( rint(x+radius*cos(a)) , rint(y+radius*sin(a)) , rint(x+radius*cos(a+.5)) , rint(y+radius*sin(a+.5)) );	//for the outside line
		if (fi)	for (float a=0; a<=pi/4; a+=.01) {	//drawing a filled circle using rectangles
			circlerectangle1.x = x-radius*(cos(a)+.015), circlerectangle1.y = y-radius*(sin(a)+.015), circlerectangle1.w = 2*radius*(cos(a)+.015), circlerectangle1.h = 2*radius*(sin(a)+.015);
			circlerectangle2.x = x-radius*(sin(a)+.015), circlerectangle2.y = y-radius*(cos(a)+.015), circlerectangle2.w = 2*radius*(sin(a)+.015), circlerectangle2.h = 2*radius*(cos(a)+.015);
			SDL_RenderFillRect(ren,&circlerectangle1);
			SDL_RenderFillRect(ren,&circlerectangle2);
			}
		}
	draw() {draw(fill);}	

	u(float dt= 1/60) {	//update
		//bouncing from borders
		if (x-radius<0) vx*=-1, x=radius; if (y-radius<0) vy*=-1,y=radius;
		if (x+radius>sx) vx*=-1, x=sx-radius; if (y+radius>sy) vy*=-1, y=sy-radius;

		x += vx*dt + ax*dt*dt/2 , y += vy*dt + ay*dt*dt/2;		vx += ax*dt, vy +=ay*dt;
		ax=0, ay=0;
		;}

	float operator-(circle &c) {	return pow(pow(c.x-x,2)+pow(c.y-y,2),.5);	}	//distance between 2 circles
	float* operator/(circle &c) {	return new float[2] {c.x-x, c.y-y};	}	//relative position vector of this circle with respect to another circle
	};

class co {public:	ve<circle> &ces;	int circlei1,circlei2;		float col;	//co:connection		circlei_: circle index 	col: connection length
	co(ve<circle>&ces, int circlei1, int circlei2) : ces(ces), circlei1(circlei1),circlei2(circlei2), col(ces[circlei1]-ces[circlei2]) {}
	draw() {	drawline(ces[circlei1].x,ces[circlei1].y,ces[circlei2].x,ces[circlei2].y);	}
	circle* getcirclep(char i=1) {	return ces.data() + (i==1?circlei1:circlei2);	}	//getcircle pointer of circlei##i
	};

class circles {public:
	ve<circle> ces;	ve<co> cons;	//ces:circles	cons: connections
	float udx,udy,udvx,udvy,	udi,m1,m2,	dis,vu,v1u,v2u,dv1u,dv2u,dxx,dyy,du,v1x,v2x,v1y,v2y, x1,x2,y1,y2,	fc=0;	//	fc: global friction const	the rest is for update

	circles(){}
	draw() {	//draw circles and connections
		for (auto it = ces.begin(); it != ces.end(); it++)	it->draw();		
		setcolor(coc);
		for (auto it = cons.begin(); it != cons.end(); it++)	it->draw();
		}
	u(float dt= 1/60) {	//update every circle	->	handle collisions	->	handle cnnections	->	draw()

		//to be optimized
		for (auto it = ces.begin(); it != ces.end(); it++)	it->ax -= fc*it->vx, it->ay -= fc*it->vy,	it->u(dt);	//applies global friction and updates every circle

		for (auto it1 = ces.begin(); it1 != ces.end(); it1++)	for (auto it2 = it1+1; it2 != ces.end(); it2++) {	//after each collision it makes sure that the collided circles are not intersecting
			for (auto it3 = cons.begin(); it3 != cons.end(); it3++) if (it3->getcirclep(1) == &*it1 && it3->getcirclep(2) == &*it2 || it3->getcirclep(1) == &*it2 && it3->getcirclep(2) == &*it1)goto nc;	//checks if *it1 is (rigidly) connected to *it2, then no collision 
			if ((udi= *it1-*it2) < it1->radius + it2->radius && ((udvx = it1->vx-it2->vx)*(udx = it1->x-it2->x)<0 || (udvy = it1->vy-it2->vy)*(udy = it1->y-it2->y)<0))
				udvy= it1->vy-it2->vy,	udy=it1->y-it2->y,
				it1->vx -= 2 * it2->mass / (it1->mass+it2->mass) * (udvx*udx + udvy*udy) / udi/udi *udx,
				it2->vx += 2 * it1->mass / (it2->mass+it1->mass) * (udvx*udx + udvy*udy) / udi/udi *udx,
				it1->vy -= 2 * it2->mass / (it1->mass+it2->mass) * (udvy*udy + udvx*udx) / udi/udi *udy,
				it2->vy += 2 * it1->mass / (it2->mass+it1->mass) * (udvy*udy + udvx*udx) / udi/udi *udy,
				it1->x += udx/udi * (it1->radius+it2->radius-udi) * it1->radius / (it1->radius+it2->radius),
				it1->y += udy/udi * (it1->radius+it2->radius-udi) * it1->radius / (it1->radius+it2->radius),
				it2->x -= udx/udi * (it1->radius+it2->radius-udi) * it2->radius / (it1->radius+it2->radius),
				it2->y -= udy/udi * (it1->radius+it2->radius-udi) * it2->radius / (it1->radius+it2->radius),
				(rui%3==0) ? aco(it1-ces.begin(), it2-ces.begin()) : false;	//connects any circle that collide with 1/2 probability
				nc:;	//no collision
			}

		for (auto it = cons.begin(); it != cons.end(); it++) {	//connection handling
			circle &circle1 = *it->getcirclep(1),	&circle2 = *it->getcirclep(2);
			x1 = circle1.x, x2 = circle2.x, y1 = circle1.y, y2 = circle2.y;
			v1x = circle1.vx, v2x = circle2.vx, v1y = circle1.vy, v2y = circle2.vy;
			dis= circle1-circle2;	du= dis-it->col;	udx = x2-x1, udy = y2-y1;

			if (dis==0) dyy=dxx=du/3;	//unexpected situation when the connected circles have the same position
			else if (it->col == 0) circle1.vx = circle2.vx = (v1x+v2x)/2, circle1.vy = circle2.vy = (v1y+v2y)/2;	//unexpected: if the connection has no length
			else {		//expected situation
				m1 = circle1.mass, m2=circle2.mass;
				v1u= (circle1.vx*udx + circle1.vy*udy) / dis, v2u= (circle2.vx*udx + circle2.vy*udy) / dis;	vu = (m1*v1u+m2*v2u)/(m1+m2);	dv1u=vu-v1u, dv2u=vu-v2u;
				circle1.vx += udx*dv1u/dis;	circle2.vx += udx*dv2u/dis;	circle1.vy += udy*dv1u/dis;	circle2.vy += udy*dv2u/dis;
				dxx=du*udx/dis/1.2; dyy=du*udy/dis/1.2;
				}

			circle1.x+=dxx, circle2.x-=dxx, circle1.y+=dyy, circle2.y-=dyy;
			;}

		draw();	//draw everything
		}

	circle &operator[](int circlei) {	return ces[circlei];	}

	//a: add
	circle* acircle(float x, float y, int radius, color cc=cdc) {	ces.emplace_back(x,y,radius,cc); return ces.data() + ces.size() - 1;	}	//returns the added circle's index
	aco(int circlei1, int circlei2) {	if (circlei1!=circlei2)	cons.emplace_back(ces,circlei1,circlei2);	}	//add connection by indices 
	};


main (int argv, char** args) {int xx1; int ai2i,ai1i;
	;SDL_Init(SDL_INIT_VIDEO); SDL_Window *win = SDL_CreateWindow("SDL2 app", 100, 100, sx,sy ,SDL_WINDOW_SHOWN); ren= SDL_CreateRenderer(win, -1, SDL_RENDERER_PRESENTVSYNC);	//preparation

	circles ccc;	circle*circlep;

	int circlecount=50;	for (auto c=circlecount;c--;)	circlep = ccc.acircle(rui%sx,rui&sy,rui%9+7),	circlep->vx = int(rui%444)-222, circlep->vy = int(rui%444)-222;	//adds 22 random circles

	int tir=11;	//tir: timer radius
	float tcounter=0, dt=1/60.;	//tcounter is the time passed since this line is executed.	dt is the resolution of our "update"s. We could do it faster by reducing dt, or making several updates in a single loop step. My screen is 60 FPS; should it be 144, I'd make dt 1/144 (seconds)

	SDL_Event ev;	//to not make the program seem like in a not responding state, and to get the keyboard input properly since the SDL_PollEvent fn. takes events such as keyboard events...
	while(1)
	{
		SDL_PollEvent(&ev); if(ev.type == SDL_QUIT) break;	//quit properly when desired

		if (SDL_GetTicks()%int(dt*1000) == 0){
			setcolor(baccolor);	SDL_RenderClear(ren);	//clears the renderer with background color
			setcolor(tic),	drawline(22,22,22+(cos(tcounter)*tir),22+(tir*sin(tcounter)));	//time indicator on the top-left. If this small line rotates smoothly clockwise, then the program runs without a problem or a lag.
			ccc.u(dt);	//update everything, THE MAIN OBJECTIVE OF THE PROGRAM
			SDL_RenderPresent(ren);	//render the visual output to screen
			tcounter+=dt;
		}}
	SDL_Quit();return 0;}
