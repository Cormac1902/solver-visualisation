#include "Display.h"

using namespace std;

int main(int argc, char *argv[]) {
//    glutInitWindowSize(800, 600);
//    glutInit(&argc, argv);
//    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
//    glutCreateWindow("3D CNF Visualization (C) 2006 C. Sinz, JKU Linz");
    Display::init(argv[1]);
//    glutDisplayFunc(display);
//    glutMotionFunc(motion);
//    glutMouseFunc(mouse);
//    glutReshapeFunc(reshape);
//    glutIdleFunc(idle);
//    glutKeyboardFunc(keyboard);
//    glutMainLoop();
    return EXIT_SUCCESS;
}
