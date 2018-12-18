/*
g++ -I/mnt/nixbig/downloads/eigen-3.3.5-eigen-b3f3d4950030/ planner_run.cpp  -o planner_run.out

g++ -std=c++11 -I/usr/include/eigen3 -L/usr/local/lib  -o planner_run planner_run.cpp -lglut -lGLU -lGL -lGLEW
*/

#include "planner.cpp"

#include <random>

#include <Eigen/Dense>

#include <GL/glut.h>

#ifndef WIN32
#include <unistd.h>
#else
#define random rand
#define srandom srand
#endif
// #define random_int random
// #define random_seed srandom


float view_translation_x = 0.0f;
float view_translation_y = 0.0f;
float view_rotation_cam_x = 0.0f;
float view_rotation_cam_y = 0.0f;
float view_rotation_cam_z = 0.0f;
/* ARGSUSED1 */
void
keyboard(unsigned char ch, int x, int y)
{
    std::cout << "keyboard: " << ch << std::endl;
  switch (ch) {
//   case ' ':
    // if (!moving) {
    //   tick();
    //   glutPostRedisplay();
    // }
    // break;
  case 56: // 8
    view_translation_y +=  1.0f;
    break;
  case 52: // 4  
    view_translation_x += -1.0f;
    break;
  case 54: // 6
    view_translation_x +=  1.0f;
    break;
  case 50: // 2
    view_translation_y += -1.0f;
    break;    
  case 55:  // 7
    view_rotation_cam_x -= 1.0f;
    break;      
  case 57:  // 9
    view_rotation_cam_x += 1.0f;
    break;    
  case 49:  // 1
    view_rotation_cam_y -= 1.0f;
    break;      
  case 51:  // 3
    view_rotation_cam_y += 1.0f;
    break;    
  case 123:  // {
    view_rotation_cam_z -= 1.0f;
    break;      
  case 125:  // }
    view_rotation_cam_z += 1.0f;
    break;    
  case 27:             /* ESC : https://theasciicode.com.ar/ascii-control-characters/escape-ascii-code-27.html */
    exit(0);
    break;
  }
  glutPostRedisplay();

}


int row_min=0; int col_min=0;
int row_max=5; int col_max=15;


Eigen::ArrayXXf plan_in(row_max,col_max);

void draw(void) {
    float red=1.0,green=1.0,blue=1.0;

    glClear(GL_DEPTH_BUFFER_BIT);
    /* paint black to blue smooth shaded polygon for background */
        glDisable(GL_DEPTH_TEST);
        glShadeModel(GL_SMOOTH);
        glBegin(GL_POLYGON);
        glColor3f(0.0, 0.0, 0.0);
        glVertex3f(-20, 20, -19);
        glVertex3f(20, 20, -19);
        glColor3f(0.0, 0.0, 1.0);
        glVertex3f(20, -20, -19);
        glVertex3f(-20, -20, -19);
        glEnd();
    /* end: paint black to blue smooth shaded polygon for background */
    /* paint surface */
        glEnable(GL_DEPTH_TEST);
        glShadeModel(GL_SMOOTH);

        glPushMatrix();
        // glTranslatef(0.0, 0.0, 0.0);
        // glTranslatef(0.0f, 0.0f, -6.0f);
        std::cout << "view_translation_x, view_translation_y = " << view_translation_x << "," << view_translation_y << std::endl;
        glTranslatef(view_translation_x, view_translation_y, -12.0f);
        glRotatef(view_rotation_cam_x,0,0,1);
        glRotatef(view_rotation_cam_y,0,1,0);
        glRotatef(view_rotation_cam_z,1,0,0);
        glTranslatef(-row_max/2.0f,col_max/2.0f,0.0f);
        // glRotatef(290.0, 1.0, 0.0, 0.0);

        glBegin(GL_TRIANGLES);


        glColor3f(0.9 * red, 0.1 * green, 0.9 * blue);
        // glVertex3f(0.0,0.0,2.0);
        // glVertex3f(1.0,0.0,2.0);
        // glVertex3f(1.0,1.0,2.0);

        // glColor3f(0.9 * red, 0.1 * green, 0.1 * blue);
        // glVertex3f(0.0,0.0,2.0);
        // glVertex3f(1.0,1.0,2.0);
        // glVertex3f(0.0,1.0,2.0);

        // glColor3f(0.9 * red, 0.1 * green, 0.1 * blue);
        // glVertex3f(0.0,0.0,2.0);
        // glVertex3f(3,3,2.0);
        // glVertex3f(0.0,3,2.0);

        Eigen::ArrayXXf neighbourhood(3,3);

        for(int row_ = row_min; row_ < row_max-1 ; row_++) {
            for(int col_ = col_min; col_ < col_max-1 ; col_++) {
            // std::cout << "\n----------\n" << neighbourhood << std::endl;
            // std::cout << "\n----------\n("<<row_<<","<<col_<<") ~~ ("<<row_+1<<","<<col_+1<<") : centred at ("<<row_+2<<","<<col_+2<<") == " << neighbourhood(1,1) << "\n" << neighbourhood /*plan_in.block<3,3>(row_,col_)*/ << std::endl;

            // glColor3f( (1.0/row_max) * row_ * red, 0.1 * green, (1.0/col_max) * col_ * blue);
            glColor3f( 1-0.1*plan_in(row_,col_), 0.1, plan_in(row_,  col_));     
            glVertex3f(row_,  -col_,  0.1*plan_in(row_,  col_));
            glColor3f( 1.0-0.1*plan_in(row_+1,col_+1), 0.1, plan_in(row_+1,  col_+1));     
            glVertex3f(row_+1,-col_-1,0.1*plan_in(row_+1,col_+1));
            glColor3f( 1.0-0.1*plan_in(row_,  col_+1), 0.1, plan_in(row_,  col_+1));     
            glVertex3f(row_,  -col_-1,0.1*plan_in(row_,  col_+1));
            
            // glColor3f( (1.0/row_max) * row_ * red, 0.5 * green, (1.0/col_max) * col_ * blue);            
            glColor3f( 1-0.1*plan_in(row_,col_), 0.1, 0.1*plan_in(row_,  col_));            
            glVertex3f(row_,  -col_,  0.1*plan_in(row_,  col_));
            glColor3f( 1-0.1*plan_in(row_+1,col_), 0.1, 0.1*plan_in(row_+1,  col_));            
            glVertex3f(row_+1,-col_,  0.1*plan_in(row_+1,col_));
            glColor3f( 1-0.1*plan_in(row_+1,col_+1), 0.1, plan_in(row_+1,  col_+1));            
            glVertex3f(row_+1,-col_-1,0.1*plan_in(row_+1,col_+1));

        }}


        glEnd();

        glPopMatrix(); 
    /* end paint surface */
  glutSwapBuffers();
}



int main(int argc, char  *argv[])
{
    vos::planning::astar_hybrid::Planner a_planner;
    a_planner.verify();
    
    Eigen::ArrayXXf plan_out;
    Eigen::Array3f start_posn;
    Eigen::Array3f goal_posn;
    plan_out = a_planner.plan(plan_in, start_posn, goal_posn);

    //--- start - Eigen checks -----------------------
        std::cout << plan_in << std::endl;
        std::cout << "-------------------------" << std::endl;
        plan_in(0,0) = 90000;
        plan_in(0,1) = 90001;
        plan_in(1,0) = 90010;
        plan_in(1,1) = 90011;
        std::cout << plan_in << std::endl;
        std::cout << "-------------------------" << std::endl;
        for(int row_ = 0; row_ < row_max ; row_++) {
            for(int col_ = 0; col_ < col_max ; col_++) {
                plan_in(row_,col_) = 90000 + row_*10 + col_;
        }}
        std::cout << plan_in << std::endl;
        std::cout << "-------------------------" << std::endl;
        srand(getpid());
        for(int row_ = 0; row_ < row_max ; row_++) {
            for(int col_ = 0; col_ < col_max ; col_++) {
                plan_in(row_,col_) = rand();
        }}
        std::cout << plan_in << std::endl;
        std::cout << "-------------------------" << std::endl;
        std::random_device randomisation_source;
        std::mt19937 a_mersenne_twister(randomisation_source());
        // std::uniform_real<float> uniform_real_dist(1.0f,10.0f);
        std::uniform_real_distribution<double> uniform_real_dist(1.0,10.0);
        for(int row_ = 0; row_ < row_max ; row_++) {
            for(int col_ = 0; col_ < col_max ; col_++) {
                plan_in(row_,col_) = uniform_real_dist(a_mersenne_twister); 
        }}
        std::cout << plan_in << std::endl;
        std::cout << "-------------------------" << std::endl;
            // std::cout << plan_in.block<3,3>(row_max,col_max) << std::endl;
        Eigen::ArrayXXf neighbourhood(3,3);
        Eigen::MatrixXf m(40,40);
        std::cout << m.block<2,2>(10,10) << std::endl;

        for(int row_ = row_min; row_ < row_max-3+1 ; row_++) {
            for(int col_ = col_min; col_ < col_max-3+1 ; col_++) {
            neighbourhood = plan_in.block<3,3>(row_,col_);
            // std::cout << "\n----------\n" << neighbourhood << std::endl;
            std::cout << "\n----------\n("<<row_<<","<<col_<<") ~~ ("<<row_+1<<","<<col_+1<<") : centred at ("<<row_+2<<","<<col_+2<<") == " << neighbourhood(1,1) << "\n" << neighbourhood /*plan_in.block<3,3>(row_,col_)*/ << std::endl;
        }}

        // for(int row_ = row_min+1; row_ < row_max-1 ; row_++) {
        //     for(int col_ = col_min+1; col_ < col_max-1 ; col_++) {
        //     neighbourhood = plan_in.block<3,3>(row_,col_);
        //     std::cout << "\n----------\n" << neighbourhood << std::endl;
        // }}
        std::cout << plan_in << std::endl;
    std::cout << "--- end   - Eigen checks -----------------------" << std::endl;
    //--- end   - Eigen checks -----------------------

    //--- start - OpenGL system setup -----------------------    
    glutInit(&argc, argv);
    /* use multisampling if available */
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH | GLUT_MULTISAMPLE);
    glutCreateWindow("planner_run");
    glutKeyboardFunc(keyboard);
    glutDisplayFunc(draw);    
    //--- end   - OpenGL system setup -----------------------

    //--- start - OpenGL view setup -----------------------
        glClearDepth(1.0);
        glClearColor(0.0, 0.0, 0.0, 0.0);
        glMatrixMode(GL_PROJECTION);
        glFrustum(-1.0, 1.0, -1.0, 1.0, 1.0, 20);
        glMatrixMode(GL_MODELVIEW);
    //--- end   - OpenGL view setup -----------------------

    
    //--- start - OpenGL run -----------------------
    glutMainLoop();
    //--- end   - OpenGL run -----------------------

    return 0;
}