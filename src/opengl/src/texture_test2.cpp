#include <GL/glut.h>
#include <stdlib.h>
#include <stdio.h>

#define TEX_HEIGHT 300
#define TEX_WIDTH 300
static GLubyte image[TEX_HEIGHT][TEX_WIDTH][4];

void initTexture(void)
{
    FILE *fp;
    int x, z;

    /* texture file open */
    if ((fp = fopen("/home/takeyama/workspace/ros2_eyeexplorer/src/opengl/image/aoffj-z2xae.tga", "rb")) == NULL)
    {
        fprintf(stderr, "texture file cannot open\n");
        return;
    }
    fseek(fp, 18, SEEK_SET);
    for (x = 0; x < TEX_HEIGHT; x++)
    {
        for (z = 0; z < TEX_WIDTH; z++)
        {
            image[x][z][2] = fgetc(fp); /* B */
            image[x][z][1] = fgetc(fp); /* G */
            image[x][z][0] = fgetc(fp); /* R */
            image[x][z][3] = fgetc(fp); /* alpha */
        }
    }
    fclose(fp);
}

void displayTexPolygon(void)
{
    glEnable(GL_TEXTURE_2D);
    glBegin(GL_QUADS);
    glTexCoord2f(0.0, 0.0);
    glVertex3f(-10.0, -10.0, 0.0);
    glTexCoord2f(0.0, 1.0);
    glVertex3f(-10.0, 10.0, 0.0);
    glTexCoord2f(1.0, 1.0);
    glVertex3f(10.0, 10.0, 0.0);
    glTexCoord2f(1.0, 0.0);
    glVertex3f(10.0, -10.0, 0.0);
    glEnd();
    glDisable(GL_TEXTURE_2D);
}

void display(void)
{
    static float spin = 0.0;
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glPushMatrix();
    {
        glTranslatef(0.0, 0.0, -20.0);
        glRotatef(spin, 0.0, 1.0, 0.0);
        glColor3f(1.0, 1.0, 1.0);
        displayTexPolygon();
    }
    spin += 1.0;
    glPopMatrix();
    glFlush();
    glutSwapBuffers();
}

void init(void)
{
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_DEPTH_TEST);
    initTexture();
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, TEX_WIDTH, TEX_HEIGHT,
                 0, GL_RGBA, GL_UNSIGNED_BYTE, image);
}

void reshape(int w, int h)
{
    glViewport(0, 0, (GLsizei)w, (GLsizei)h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(-5.0, 5.0, -5.0, 5.0, 5.0, 500.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

int main(int argc, char **argv)
{
    // GLUTの初期化
    glutInit(&argc, argv);
    // GLUTのディスプレイモード設定
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    // ウィンドウのサイズ
    glutInitWindowSize(500, 500);
    // ウィンドウの位置
    glutInitWindowPosition(100, 100);
    // ウィンドウの生成
    glutCreateWindow(argv[0]);

    // 初期化
    init();

    
    glutIdleFunc(display);
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);

    // この関数は、GLUT がイベント処理ループに入ることを表します
    // この関数を呼び出せば、トップレベル ウィンドウが破棄されるまで処理は戻ってきません
    glutMainLoop();
    return 0;
}