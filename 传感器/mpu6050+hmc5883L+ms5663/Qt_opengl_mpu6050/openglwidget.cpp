#include "openglwidget.h"
#include "qdebug.h"



OpenglWidget::OpenglWidget(QWidget *parent):QGLWidget(parent)
{
   initWidget();
   initializeGL();
   b_x=false;
   b_y=false;
   b_z=false;

//   m_FileName = "D:/tgl/qt_project/lesson07/lesson07/dataFont1.tga";       //应根据实际存放图片的路径进行修改

}

void OpenglWidget::initializeGL()
{

  glShadeModel(GL_SMOOTH);
  glClearColor(0.0,0.0,0.0,0.5);
  glClearDepth(1.0);
  glEnable(GL_DEPTH_TEST);

  glDepthFunc(GL_LEQUAL);
  glHint(GL_PERSPECTIVE_CORRECTION_HINT,GL_NICEST);

//  glColor4f( 1.0, 1.0, 1.0, 0.5 );
//  glBlendFunc( GL_SRC_ALPHA, GL_ONE );
    glBlendFunc( GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA );
  glEnable(GL_BLEND);


}

void OpenglWidget::initWidget()
{
   setGeometry(400,200,640,480);
   setWindowTitle("OpenGL demo");

}





void OpenglWidget::paintGL()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();

   glTranslatef(-0.5,0.0,-6.0);
   glRotatef(-20.0,0.0,1.0,0.0);

   draw3DCordiate();


    if(b_x)
    {
    //  b_x=false;
      glRotatef(m_xrotateRectangle,1.0,0.0,0.0);

    }

    if(b_y)
    {
      // b_y = false;
       glRotatef(m_yrotateRectangle,0.0,1.0,0.0);
    }

    if(b_z)
    {
    //  b_z = false;
      glRotatef(m_zrotateRectangle,0.0,0.0,1.0);
    }

    draw3DCubeObj();

 }



void OpenglWidget::resizeGL(int w, int h)
{
   if(0==h)
   {
       h = 1;
   }
   qDebug()<<"the debug info w: "<<w<<"  h: "<<h;
   glViewport(0,0,(GLint)w,(GLint)h);
   glMatrixMode(GL_PROJECTION);

   glLoadIdentity();

   GLdouble aspectRatio = (GLfloat)w/(GLfloat)h;
   GLdouble zNear = 0.1;// 0.1;
   GLdouble zFar = 100.0;

   GLdouble rFov = 45.0 * 3.14159265/180.0;
   glFrustum(-zNear*tan(rFov/2.0)*aspectRatio,zNear*tan(rFov/2.0)*aspectRatio,
             -zNear*tan(rFov/2.0  ),zNear*tan(rFov/2.0),zNear,zFar);

   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
}

void OpenglWidget::x_rotate(float value_angle)
{
      if(value_angle == 0.0)
      {
         b_x = false;
      }
      else
      {
          b_x = true;
      }

    m_xrotateRectangle = value_angle;
    update();
}

void OpenglWidget::y_rotate(float value_angle)
{
    if(value_angle == 0.0)
    {
        b_y=false;
    }
    else
    {
        b_y = true;
    }
    m_yrotateRectangle = value_angle;
    update();
}

void OpenglWidget::z_rotate(float value_angle)
{
    if(value_angle == 0.0)
    {
        b_z = false;
    }
    else
    {
       b_z = true;
    }

    m_zrotateRectangle = value_angle;
    update();
}

void OpenglWidget::draw3DCordiate()
{
    setFont(QFont("Times",24));
    //x轴
    glLineWidth(3.0);
    glBegin(GL_LINES);
    glColor3f(1.0f,0.0f,0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(2.0, 0.0f, 0.0f);
    glEnd();

    glPushMatrix();
    glTranslatef(2.0, 0.0f, 0.0f);
    glRotatef(90.0f,0.0f,1.0f,0.0f);
    glutWireCone(0.054,0.15,10,10);
    glPopMatrix();

    renderText(2.2,0,0,QChar('x'));

    //y轴
    glBegin(GL_LINES);
     glColor3f(0.0f,1.0f,0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0, 2.0f, 0.0f);
    glEnd();

    glPushMatrix();
    glTranslatef(0.0, 2.0f, 0.0f);
    glRotatef(270.0f,1.0f,0.0f,0.0f);
    glutWireCone(0.054,0.15,10,10);
    glPopMatrix();

     renderText(0.1,2.1,0,QChar('z'));

    //z轴
    glBegin(GL_LINES);
     glColor3f(0.0f,0.0f,1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0, 0.0f,2.0f);
    glEnd();

    glPushMatrix();
    glTranslatef(0.0, 0.0f, 2.0f);
    glRotatef(0.0f,1.0f,0.0f,0.0f);
    glutWireCone(0.054,0.15,10,10);
    glPopMatrix();

     renderText(0,0.1,2.1,QChar('y'));
}

void OpenglWidget::draw3DCubeObj()
{
    glBegin(GL_QUADS);

  //  glColor3f(1.0,0.0,0.0);
    glColor4f(1.0,0.0,0,0.6);
    glVertex3f(-1,1,1); glVertex3f(-1,1,-1);glVertex3f(-1,-1,-1);glVertex3f(-1,-1,1);

  //  glColor3f(0.0,0.0,1.0);
    glColor4f(0.0,1,0.0,0.6);
    glVertex3f(-1,1,-1); glVertex3f(-1,-1,-1);glVertex3f(1,-1,-1);glVertex3f(1,1,-1);

  //  glColor3f(0.0,1.0,0.0);
    glColor4f(0,0,1,0.6);
    glVertex3f(1,1,-1); glVertex3f(1,-1,-1);glVertex3f(1,-1,1);glVertex3f(1,1,1);

  //  glColor3f(0.1,0.5,0.7);
    glColor4f(0.8,0.2,0.0,0.6);
    glVertex3f(1,-1,1); glVertex3f(1,1,1);glVertex3f(-1,1,1);glVertex3f(-1,-1,1);

  //  glColor3f(0.8,0.6,0.2);
    glColor4f(0,2.8,0.2,0.6);
    glVertex3f(-1,1,1); glVertex3f(1,1,1);glVertex3f(1,1,-1);glVertex3f(-1,1,-1);

  //  glColor3f(0.6,0.8,0.2);
    glColor4f(0.2,0,0.8,0.6);
    glVertex3f(-1,-1,1); glVertex3f(-1,-1,-1);glVertex3f(1,-1,-1);glVertex3f(1,-1,1);


    glEnd();

}

void OpenglWidget::drawString(const char *str)
{
    static int isFirstCall = 1;
    static GLuint lists;

    if(isFirstCall)
    {
        isFirstCall = 0;
        lists= glGenLists(MAX_CHAR);
        wglUseFontBitmaps(wglGetCurrentDC(),0,MAX_CHAR,lists);
    }

    for(;*str!='/0';++str)
        glCallList(lists+*str);
}

void OpenglWidget::display()
{
  glClear(GL_COLOR_BUFFER_BIT);
  glColor3f(1.0f,0.0f,0.0f);
  glRasterPos2f(0.0f,0.0f);
  drawString("hello world");

  glutSwapBuffers();

}

void OpenglWidget::buildFont()
{
    float cx, cy;                                       //储存字符的x、y坐标
    m_Base = glGenLists(256);                           //创建256个显示列表
    glBindTexture(GL_TEXTURE_2D, m_Texture);            //选择字符纹理

    for (int i=0; i<256; i++)                           //循环256个显示列表
    {
        cx = float(i%16)/16.0f;                         //当前字符的x坐标
        cy = float(i/16)/16.0f;                         //当前字符的y坐标

        glNewList(m_Base+i, GL_COMPILE);                //开始创建显示列表
        glBegin(GL_QUADS);                          //使用四边形显示每一个字符
                  glTexCoord2f(cx, 1.0f-cy-0.0625f);
                  glVertex2i(0, 16);
                  glTexCoord2f(cx+0.0625f, 1.0f-cy-0.0625f);
                  glVertex2i(16, 16);
                  glTexCoord2f(cx+0.0625f, 1.0f-cy);
                  glVertex2i(16, 0);
                  glTexCoord2f(cx, 1.0f-cy);
                  glVertex2i(0, 0);
         glEnd();                                    //四边形字符绘制完成
              glTranslated(14, 0, 0);                     //绘制完一个字符，向右平移14个单位
         glEndList();                                    //字符显示列表完成
      }

}

void OpenglWidget::killFont()
{
   glDeleteLists(m_Base, 256);                         //删除256个显示列表
}

void OpenglWidget::glPrint(GLuint x, GLuint y, int set, const char *fmt,...)
{
    char text[1024];                                    //保存字符串
    va_list ap;                                         //指向一个变量列表的指针

    if (fmt == NULL)                                    //如果无输入则返回
    {
        return;
    }

    va_start(ap, fmt);                                  //分析可变参数
        vsprintf(text, fmt, ap);                        //把参数值写入字符串
    va_end(ap);                                         //结束分析

    if (set > 1)                                        //如果字符集大于1
    {
        set = 1;                                        //设置其为1
    }
    glEnable(GL_TEXTURE_2D);                            //启用纹理
    glLoadIdentity();                                   //重置模型观察矩阵
    glTranslated(x, y ,0);                              //把字符原点移动到(x,y)位置
    glListBase(m_Base-32+(128*set));                    //选择字符集

    glScalef(1.0f, 2.0f, 1.0f);                         //如果是第一个字符集，放大字体

    glCallLists(strlen(text), GL_BYTE, text);           //把字符串写到屏幕
    glDisable(GL_TEXTURE_2D);                           //禁用纹理

}


void OpenglWidget::drawOrd()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    glTranslatef(-1.0f,-1.0f,-6.0f);

    glColor3f(1.0,0.0,0.0);

    glBegin(GL_LINES);
    glVertex3f(0.0f,0.0f,0.0f);
    glVertex3f(3,0,0);
    glVertex3f(0,0,0);
    glVertex3f(0,3,0);
    glVertex3f(0,0,0);
    glVertex3f(0,0,3);
    glEnd();


    glTranslatef(-1.0f,-1.0f,-6.0f);
    glColor3f(0.5,0.5,1.0);

    glBegin(GL_QUADS);
    glVertex3f(0,0,0);
    glVertex3f(3,0,0);
    glVertex3f(3,0,3);
    glVertex3f(0,0,3);
    glEnd();

    //X
    glTranslatef(4,0,0);
    glColor3f(0,1,0);

    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(1,-1,0);
    glVertex3f(1,0,0);
    glVertex3f(0,-1,0);
    glEnd();

    //Y
    glTranslatef(-5,4,0);
    glColor3f(0,0.5,0.5);

    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(0,-1,0);
    glVertex3f(0,0,0);
    glVertex3f(-0.5,0.5,0);
    glVertex3f(0,0,0);
    glVertex3f(0.5,0.5,0);
    glEnd();

    //Z
    glTranslatef(0,-4,4);
    glColor3f(0.5,0,0.5);

    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(0.5,0,0);
    glVertex3f(0.5,0,0);
    glVertex3f(0,-0.5,0);
    glVertex3f(0,-0.5,0);
    glVertex3f(0.5,-0.5,0);
    glEnd();


}

