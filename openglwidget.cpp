#include "openglwidget.h"

OpenGLWidget::OpenGLWidget(QWidget *parent) : QOpenGLWidget(parent), roll(0.0f), pitch(0.0f), yaw(0.0f) {
    program = new QOpenGLShaderProgram(this);
}

OpenGLWidget::~OpenGLWidget() {
    makeCurrent();  // 确保当前上下文是活跃的
    vbo.destroy();
    vao.destroy();
    program->deleteLater();
    doneCurrent();  // 完成后释放上下文
}
void OpenGLWidget::initializeGL() {
    initializeOpenGLFunctions();
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glEnable(GL_DEPTH_TEST);

    program = new QOpenGLShaderProgram(this);
    if (!program->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/vertexShader.vsh") ||
        !program->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/fragmentShader.fsh") ||
        !program->link()) {
        qDebug() << "Shader Error:" << program->log();
    }

    program->bind();

    GLfloat vertices[] = {
        // 机身 - 长方体
        -0.1f, -0.05f, -0.5f,  0.1f, -0.05f, -0.5f,  0.1f,  0.05f, -0.5f,  // 后面
         0.1f,  0.05f, -0.5f, -0.1f,  0.05f, -0.5f, -0.1f, -0.05f, -0.5f,
        -0.1f, -0.05f,  0.6f,  0.1f, -0.05f,  0.6f,  0.1f,  0.05f,  0.6f,  // 前面
         0.1f,  0.05f,  0.6f, -0.1f,  0.05f,  0.6f, -0.1f, -0.05f,  0.6f,
        -0.1f, -0.05f, -0.5f, -0.1f,  0.05f, -0.5f, -0.1f,  0.05f,  0.6f,  // 左侧
        -0.1f,  0.05f,  0.6f, -0.1f, -0.05f,  0.6f, -0.1f, -0.05f, -0.5f,
         0.1f, -0.05f, -0.5f,  0.1f,  0.05f, -0.5f,  0.1f,  0.05f,  0.6f,  // 右侧
         0.1f,  0.05f,  0.6f,  0.1f, -0.05f,  0.6f,  0.1f, -0.05f, -0.5f,
        -0.1f,  0.05f, -0.5f,  0.1f,  0.05f, -0.5f,  0.1f,  0.05f,  0.6f,  // 顶部
         0.1f,  0.05f,  0.6f, -0.1f,  0.05f,  0.6f, -0.1f,  0.05f, -0.5f,
        -0.1f, -0.05f, -0.5f,  0.1f, -0.05f, -0.5f,  0.1f, -0.05f,  0.6f,  // 底部
         0.1f, -0.05f,  0.6f, -0.1f, -0.05f,  0.6f, -0.1f, -0.05f, -0.5f,

        // 机头 - 圆锥
        0.0f, 0.0f, 0.8f, -0.1f, -0.05f, 0.6f, 0.1f, -0.05f, 0.6f,  // 底面
        0.0f, 0.0f, 0.8f, 0.1f, -0.05f, 0.6f, 0.1f, 0.05f, 0.6f,   // 右面
        0.0f, 0.0f, 0.8f, 0.1f, 0.05f, 0.6f, -0.1f, 0.05f, 0.6f,   // 顶面
        0.0f, 0.0f, 0.8f, -0.1f, 0.05f, 0.6f, -0.1f, -0.05f, 0.6f,  // 左面

        // 主翼 - 平面
        -0.5f, 0.02f, -0.1f, 0.5f, 0.02f, -0.1f, 0.5f, 0.02f, 0.1f,  // 右翼
        0.5f, 0.02f, 0.1f, -0.5f, 0.02f, 0.1f, -0.5f, 0.02f, -0.1f,  // 左翼

        // 水平尾翼
        -0.3f, -0.02f, -0.3f, 0.3f, -0.02f, -0.3f, 0.3f, -0.02f, -0.2f,
        0.3f, -0.02f, -0.2f, -0.3f, -0.02f, -0.2f, -0.3f, -0.02f, -0.3f,

        // 垂直尾翼
        0.0f, 0.1f, -0.3f, 0.0f, -0.1f, -0.3f, 0.0f, 0.0f, -0.5f,

        // 发动机 - 圆柱（简化为长方体）
        -0.15f, -0.05f, -0.2f, -0.15f, 0.05f, -0.2f, -0.15f, 0.05f, -0.4f,  // 左发动机
        -0.15f, 0.05f, -0.4f, -0.15f, -0.05f, -0.4f, -0.15f, -0.05f, -0.2f,
        0.15f, -0.05f, -0.2f, 0.15f, 0.05f, -0.2f, 0.15f, 0.05f, -0.4f,  // 右发动机
        0.15f, 0.05f, -0.4f, 0.15f, -0.05f, -0.4f, 0.15f, -0.05f, -0.2f
    };


    vao.create();
    vbo.create();
    vao.bind();
    vbo.bind();
    vbo.allocate(vertices, sizeof(vertices));

    int vertexLocation = program->attributeLocation("aPos");
    program->enableAttributeArray(vertexLocation);
    program->setAttributeBuffer(vertexLocation, GL_FLOAT, 0, 3, 0);

    vao.release();
    vbo.release();
    program->release();
}

void OpenGLWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    program->bind();
    vao.bind();

    setupTransformations();

    glDrawArrays(GL_TRIANGLES, 0, 225);

    vao.release();
    program->release();
}

void OpenGLWidget::setupTransformations() {
    projection.setToIdentity();
    projection.perspective(45.0f, float(width()) / float(height()), 0.1f, 100.0f);

    view.setToIdentity();
    view.lookAt(QVector3D(0.0, 0.0, 3.0), QVector3D(0.0, 0.0, 0.0), QVector3D(0.0, 1.0, 0.0));

    model.setToIdentity();
    model.rotate(yaw, 0.0, 1.0, 0.0);
    model.rotate(pitch, 1.0, 0.0, 0.0);
    model.rotate(roll, 0.0, 0.0, 1.0);

    program->setUniformValue("model", model);
    program->setUniformValue("view", view);
    program->setUniformValue("projection", projection);
}

void OpenGLWidget::resizeGL(int width, int height) {
    projection.setToIdentity();
    projection.perspective(45.0f, GLfloat(width) / height, 0.01f, 100.0f);
}

void OpenGLWidget::updateIMUData(float roll, float pitch, float yaw) {
    this->roll = roll;
    this->pitch = pitch;
    this->yaw = yaw;
    update();
}
