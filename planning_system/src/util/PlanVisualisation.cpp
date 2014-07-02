#include <GL/freeglut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glx.h>

#include <iostream>
#include <string>

#define WINDOW_WIDTH 1600
#define WINDOW_HEIGHT 900
#define SPACE 32

namespace KCL_rosplan {

	double largestDuration;
	int window;
	GLuint base;

	/* The function called whenever a key is pressed. */
	void keyPressed(unsigned char key, int x, int y) {

		/* avoid thrashing this procedure */
		usleep(100);

		if (key == SPACE) KCL_rosplan::dispatchPaused = !KCL_rosplan::dispatchPaused;
	}

	/*-----------*/
	/* shut down */
	/*-----------*/

	void shutDownVis() {
		glutDestroyWindow(window);
		glDeleteLists(base, 96);
	}

	/*---------------*/
	/* init graphics */
	/*---------------*/

	/* build font */
	void buildFont() {

		Display *dpy;
		XFontStruct *fontInfo;
		base = glGenLists(96);

		dpy = XOpenDisplay(NULL); // default to DISPLAY env.   

		fontInfo = XLoadQueryFont(dpy, "-adobe-helvetica-medium-r-normal--18-*-*-*-p-*-iso8859-1");
		if (fontInfo == NULL) fontInfo = XLoadQueryFont(dpy, "fixed");
		if (fontInfo == NULL) std::cout << "no X font available?" << std::endl;

		glXUseXFont(fontInfo->fid, 32, 96, base);
		XFreeFont(dpy, fontInfo);
		XCloseDisplay(dpy);
	}

	/* OpenGL initialization */
	void initVis() {

		char appname[] = "planner";
		char *fake_argv[] = { appname, NULL };
		int fake_argc = 1;
		glutInit(&fake_argc, fake_argv);
		glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA);  

		glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
		glutInitWindowPosition(0, 0);  
		window = glutCreateWindow("KCL: Planning visualisation");  

		glutKeyboardFunc(&keyPressed);

 		buildFont();

		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

		glEnable(GL_TEXTURE_2D);
		glShadeModel(GL_SMOOTH);
		glDisable(GL_DEPTH_TEST);
		glDisable(GL_LIGHTING);

		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glEnable(GL_ALPHA_TEST);
		glAlphaFunc(GL_GREATER, 0.1f);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameterf( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
			 
		glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
		glMatrixMode(GL_MODELVIEW);

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(0, WINDOW_WIDTH, WINDOW_HEIGHT, 0, 1, -1);
		glMatrixMode(GL_MODELVIEW);
	}

	/*--------------*/
	/* draw methods */
	/*--------------*/

	void drawText(const std::string &textstring, int size, float x, float baseline, float r, float g, float b) {

		const char* text = textstring.c_str();

		glPushMatrix();
		glDisable(GL_TEXTURE_2D);
		glLoadIdentity();

		glColor4f(r,g,b,1.0f);
		glTranslatef(x, baseline+size, 0.0);
		glRasterPos2f(0.0, 0.0);

		int length = (int) strlen(text);
		for (int i = 0; i < length; i++) {
			if(size==12) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, text[i]);
			if(size==18) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, text[i]);
		}

		glEnable(GL_TEXTURE_2D);
		glPopMatrix();
	}

	void drawRect(float x, float y, float width, float height, float r, float g, float b) {

		glPushMatrix();
		glBindTexture(GL_TEXTURE_2D, 0);

		glColor4f(r,g,b,1.0f);
		glTranslatef(x, y, 0);		

		glBegin(GL_LINE_LOOP);
		{
			glVertex2f(0, 0);
			glVertex2f(0, height);
			glVertex2f(width,height);
			glVertex2f(width,0);
		}
		glEnd();
		glPopMatrix();
	}

	void fillRect(float x, float y, float width, float height, float r, float g, float b) {

		glPushMatrix();
		glBindTexture(GL_TEXTURE_2D, 0);

		glColor4f(r,g,b,1.0f);
		glTranslatef(x, y, 0);		

		glBegin(GL_POLYGON);
		{
			glVertex2f(0, 0);
			glVertex2f(0, height);
			glVertex2f(width,height);
			glVertex2f(width,0);
		}
		glEnd();
		glPopMatrix();
	}

	void drawAction(planning_dispatch_msgs::ActionDispatch &msg, float startTime, size_t level) {

		bool currentPlan = (level == KCL_rosplan::planningAttempts-1);

		float r, g, b;
		if((size_t)msg.action_id < KCL_rosplan::currentAction) {
			r = (25.0/255.0f); g = (25.0/255.0f); b = (112.0/255.0f);
		}
		else if(KCL_rosplan::currentAction == (size_t)msg.action_id) {
			if(currentPlan) { r = 0.0; g = (205.0/255.0f); b = 0.0; }
			else { r = (122.0/255.0f); g = (25.0/255.0f); b = (25.0/255.0f); }
		}
		else {
			if(currentPlan) { r = 0.0; g = 0.0; b=0.0; }
			else { r = (122.0/255.0f); g = (25.0/255.0f); b = (25.0/255.0f); }
		}

		float x = WINDOW_WIDTH * startTime / largestDuration;
		float width = WINDOW_WIDTH * msg.duration / largestDuration;

		int barHeight = 25;
		fillRect(x, level*(barHeight+14), width, barHeight, r,g,b);
		drawRect(x, level*(barHeight+14), width, barHeight, 1.0,1.0,1.0);
		drawText(msg.name, 12, x+2, level*(barHeight+14) + barHeight, 0.1f, 0.1f, 0.1f);

		if(KCL_rosplan::currentAction == (size_t)msg.action_id && currentPlan) {

			int text_baseline = 100 + (barHeight+14)*KCL_rosplan::planningAttempts;
			std::stringstream ss;
			ss << msg.action_id << ": " << msg.name << "[" << msg.duration << "]";
			drawText(ss.str(), 18, 20, text_baseline, 0.0,0.0,0.0);
			text_baseline += 20;

			for(size_t i=0;i<msg.parameters.size();i++) {
				std::stringstream ss_param;
				ss_param << msg.parameters[i].key.c_str() << " -> " << msg.parameters[i].value.c_str();
				drawText(ss_param.str(), 12, 70, text_baseline, 0.1f, 0.1f, 0.1f);
				text_baseline += 12;
			}
		}
	}

	/* The main drawing function */
	void draw() {

		if(KCL_rosplan::totalPlanDuration > largestDuration)
			largestDuration = KCL_rosplan::totalPlanDuration;

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glLoadIdentity();

		fillRect(0,0,WINDOW_WIDTH,WINDOW_HEIGHT, 0.8f,0.8f,0.8f);

		float time = 0.0;
		for(size_t i=0;i<KCL_rosplan::planList.size();i++) {
			time = 0.0;
			for(size_t j=0; j<KCL_rosplan::planList[i].size(); j++) {
				drawAction(KCL_rosplan::planList[i][j], time, i);
				time = time + KCL_rosplan::planList[i][j].duration;
			}
		}

		time = 0.0;
		for(size_t i=0; i<KCL_rosplan::actionList.size(); i++) {
			drawAction(KCL_rosplan::actionList[i], time, KCL_rosplan::planningAttempts-1);
			time = time + KCL_rosplan::actionList[i].duration;
		}

		if(KCL_rosplan::dispatchPaused)
			drawText("DISPATCH PAUSED", 18, 0, WINDOW_HEIGHT-20, 0.0,0.0,0.0);

		glutSwapBuffers();
		glutMainLoopEvent();
	}
}
