#ifndef OPENGLWIDGET_H
#define OPENGLWIDGET_H
#include <QtOpenGL>
#include <GL/gl.h>
#include <qgl.h>
#include <QGLWidget>
#include <GL/glu.h>
#include <windows.h>

#include <stdarg.h>


#include <glut/glut.h>

#pragma comment(lib,"glut32.lib");

#define MAX_CHAR 128


class OpenglWidget : public QGLWidget
{
public:
    OpenglWidget(QWidget *parent=0);
    void drawOrd();

protected:
    void initializeGL();
    void initWidget();
    void paintGL();
    void resizeGL(int w, int h);

public slots:
    void x_rotate(float x);
    void y_rotate(float y);
    void z_rotate(float z);

private:
    void draw3DCordiate();
    void draw3DCubeObj();
    void drawString(const char *str);
    void display(void);

    void buildFont();                               //创建字体
    void killFont();                                //删除显示列表
    void glPrint(GLuint x, GLuint y, int set, const char *fmt,...);//输出字符串


private:
//  GLfloat m_rotateRectangle;

    GLfloat m_xrotateRectangle;
    GLfloat m_yrotateRectangle;
    GLfloat m_zrotateRectangle;


    bool b_x;
    bool b_y;
    bool b_z;


    GLuint m_Base;                                  //字符显示列表的开始值
    QString m_FileName;                             //图片的路径及文件名
    GLuint m_Texture;                               //储存一个纹理

};

#endif // OPENGLWIDGET_H
