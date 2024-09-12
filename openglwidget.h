#ifndef OPENGLWIDGET_H
#define OPENGLWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QMatrix4x4>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>

class OpenGLWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    OpenGLWidget(QWidget *parent = nullptr);
    ~OpenGLWidget();

    void updateIMUData(float roll, float pitch, float yaw);  // 添加或确保此行存在

protected:
    void initializeGL() override;
    void paintGL() override;
    void setupTransformations();
    void resizeGL(int width, int height) override;

private:
    QOpenGLShaderProgram *program;
    QMatrix4x4 projection;
    //QMatrix4x4 view;  // Add this line
    QMatrix4x4 model, view;

    float roll, pitch, yaw;

    QOpenGLVertexArrayObject vao;
    QOpenGLBuffer vbo;
};

#endif // OPENGLWIDGET_H
