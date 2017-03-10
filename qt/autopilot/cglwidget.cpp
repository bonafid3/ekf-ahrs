#include "cglwidget.h"
#include <QDebug>
//

#include <qmath.h>
#include "utils.h"

#include <QString>


cGLWidget::cGLWidget(QWidget *parent) :
    QOpenGLWidget(parent), mSavedX(0), mSavedY(0), mLeftMouseButtonPressed(false),
    mPrgOutline(new QOpenGLShaderProgram), mEye(QVector3D(0,0,100)), mView(QVector3D(0,0,0)), mUp(QVector3D(0,1,0)), mPerspectiveProjection(true)
{
    setMouseTracking(true);

    QSurfaceFormat format;
    format.setSamples(4);
    setFormat(format);
}

float cGLWidget::scale(float from_min, float from_max, float to_min, float to_max, float val)
{
    return (to_max - to_min)*(val - from_min) / (from_max - from_min) + to_min;
}


void cGLWidget::initializeGL()
{
    initializeOpenGLFunctions();

    glClearColor(0.5, 0.8, 0.5, 0.5);

    //qd << "Supported shading language version"  << QString((char*)glGetString(GL_VENDOR))<< QString((char*)glGetString(GL_RENDERER)) << QString((char*)glGetString(GL_VERSION)) << QString((char*)glGetString(GL_EXTENSIONS)) << QString((char*)glGetString(GL_SHADING_LANGUAGE_VERSION));

    qDebug() << "Initializing shaders...";
    initShaders();

    // Enable depth buffer
    glEnable(GL_DEPTH_TEST);

    //glShadeModel(GL_FLAT);

    // Enable back face culling
    glEnable(GL_CULL_FACE);

    glEnable(GL_TEXTURE_2D); // Enable Texture Mapping ( NEW )

    glEnable(GL_MULTISAMPLE); // qt surface format sets the number of sampling

    //glClearDepth(1.0f); // Depth Buffer Setup
    //glDepthFunc(GL_LEQUAL); // The Type Of Depth Testing To Do

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glActiveTexture(GL_TEXTURE0);

    QImage texture(":/texture.jpg");
    if(!texture.isNull()) {
        createTexture(texture);
    }

    installEventFilter(this);




    QVector<GLfloat> data;

    add(data, QVector3D(25,25,0.01), QVector2D(0.5,0));
    add(data, QVector3D(-25,25,0.01), QVector2D(0,0));
    add(data, QVector3D(-25,-25,0.01), QVector2D(0,1));
    add(data, QVector3D(25,-25,0.01), QVector2D(0.5,1));


    add(data, QVector3D(-25,25,0.01), QVector2D(1,0));
    add(data, QVector3D(25,25,0.01), QVector2D(0.5,0));
    add(data, QVector3D(25,-25,0.01), QVector2D(0.5,1));
    add(data, QVector3D(-25,-25,0.01), QVector2D(1,1));


    createPlaneVBO(data);

    glPointSize(5.0);

    emit(glInitialized());
}

void cGLWidget::setPointScale(int scale) {
    mPointScale = scale;
}

void cGLWidget::createTexture(QImage texture)
{
    if(mTexture) delete mTexture;
    mTexture = new QOpenGLTexture(texture);
    mTexture->setMinificationFilter(QOpenGLTexture::LinearMipMapLinear);
    mTexture->setMagnificationFilter(QOpenGLTexture::Linear);
}

void cGLWidget::initShaders()
{
    // Overriding system locale until shaders are compiled
    setlocale(LC_NUMERIC, "C");

    if(!mPrgOutline->addShaderFromSourceFile(QOpenGLShader::Vertex, "d:/projects/autopilot/vshader.vert"))
    {
        qd << "error compiling vertex shader";
        close();
    }
    if (!mPrgOutline->addShaderFromSourceFile(QOpenGLShader::Fragment, "d:/projects/autopilot/fshader.frag"))
    {
        qd << "error compiling fragment shader";
        close();
    }
    mPrgOutline->bindAttributeLocation("qt_vertex", 0);
    mPrgOutline->bindAttributeLocation("qt_textureCoords", 1);

    // Linking shader pipeline
    if (!mPrgOutline->link())
        close();

    // Binding shader pipeline for use
    if (!mPrgOutline->bind())
        close();

    mPrgOutline->release();

    // Restore system locale
    setlocale(LC_ALL, "");
}

void cGLWidget::add(QVector<GLfloat> &b, QVector3D v, const QVector2D tc)
{
    b.append(v.x());
    b.append(v.y());
    b.append(v.z());
    b.append(tc.x());
    b.append(tc.y());
}



void cGLWidget::createPlaneVBO(QVector<GLfloat> &buffer)
{
    if(!buffer.size()) {
        return;
    }

    qd << mPlaneVBO.create();

    qd << mPlaneVBO.bind();
    mPlaneVBO.allocate(buffer.constData(), buffer.size() * sizeof(GLfloat));

    mPlaneVBO.release();
}

void cGLWidget::createPointVBO(QVector<GLfloat> &buffer)
{
    if(!buffer.size()) {
        return;
    }

    if(mPointVBO.isCreated()) {
        mPointVBO.destroy();
    }

    mPointVBO.create();

    qd << mPointVBO.bind();
    mPointVBO.allocate(buffer.constData(), buffer.size() * sizeof(GLfloat));

    mPointVBO.release();
}




void cGLWidget::topView()
{
    mEye = QVector3D(0,60,0);
    mView = QVector3D(0,0,0);
    mUp = QVector3D(0,0,-1);

    update();
}

void cGLWidget::leftView()
{
    mEye = QVector3D(60,0,0);
    mView = QVector3D(0,0,0);
    mUp = QVector3D(0,1,0);

    update();
}

void cGLWidget::rightView()
{
    mEye = QVector3D(-60,0,0);
    mView = QVector3D(0,0,0);
    mUp = QVector3D(0,1,0);

    update();
}

void cGLWidget::frontView()
{
    mEye = QVector3D(0,0,60);
    mView = QVector3D(0,0,0);
    mUp = QVector3D(0,1,0);

    update();
}

void cGLWidget::backView()
{
    mEye = QVector3D(60,0,60);
    mView = QVector3D(0,0,0);
    mUp = QVector3D(0,1,0);

    update();
}

void cGLWidget::resizeGL(int w, int h)
{
    updateProjection(mPerspectiveProjection);
}

void cGLWidget::updateProjection(bool perspectiveProjection)
{
    int w=width(), h=height();

    float aspect = (float) w / (float) h;

    float os = 25.;

    mProjectionMatrix.setToIdentity();
    if(perspectiveProjection) {
        mProjectionMatrix.perspective(45.0f, aspect, 0.01f, 1000.0f);
    } else {
        if(w <= h)  {
            mProjectionMatrix.ortho(-os, os, -os/aspect, os/aspect, 0.01f, 1000.0f);
        } else {
            mProjectionMatrix.ortho(-os*aspect, os*aspect, -os, os, 0.01f, 1000.0f);
        }
    }
    mPerspectiveProjection = perspectiveProjection;
}

void cGLWidget::setAttitudeQuat(QQuaternion q)
{
    mAttitudeQuat = q;
}

void cGLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    QMatrix4x4 modelMatrix;
    modelMatrix.rotate(mAttitudeQuat);

    QOpenGLFunctions *f=0;
    QMatrix4x4 viewMatrix;

    viewMatrix.lookAt(mEye, mView, mUp);

    // draw the background plane
    if(!mPlaneVBO.isCreated()) {
        mPlaneVBO.create();
    }

    mPlaneVBO.bind();
    if(mPlaneVBO.size() > 0) {

        mTexture->bind();

        mPrgOutline->bind();
        mPrgOutline->setUniformValue("qt_opacity", mOpacity);
        mPrgOutline->setUniformValue("qt_pointScale", 0);
        mPrgOutline->setUniformValue("qt_modelMatrix", modelMatrix);
        mPrgOutline->setUniformValue("qt_viewMatrix", viewMatrix);
        mPrgOutline->setUniformValue("qt_projectionMatrix", mProjectionMatrix);
        f = QOpenGLContext::currentContext()->functions();
        f->glEnableVertexAttribArray(0);
        f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(GLfloat), 0);
        f->glEnableVertexAttribArray(1);
        f->glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(GLfloat), reinterpret_cast<void *>(3 * sizeof(GLfloat)));
        glDrawArrays( GL_QUADS, 0,  mPlaneVBO.size() / 5 / sizeof(GLfloat) );

        mTexture->release();

        mPrgOutline->release();
    }
    mPlaneVBO.release();

    // draw the background plane
    if(!mPointVBO.isCreated()) {
        mPointVBO.create();
    }

    mPointVBO.bind();
    if(mPointVBO.size() > 0) {

        mPrgOutline->bind();
        mPrgOutline->setUniformValue("qt_opacity", mOpacity);
        mPrgOutline->setUniformValue("qt_pointScale", mPointScale);
        mPrgOutline->setUniformValue("qt_modelMatrix", modelMatrix);
        mPrgOutline->setUniformValue("qt_viewMatrix", viewMatrix);
        mPrgOutline->setUniformValue("qt_projectionMatrix", mProjectionMatrix);
        f = QOpenGLContext::currentContext()->functions();
        f->glEnableVertexAttribArray(0);
        f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), 0);
        //f->glEnableVertexAttribArray(1);
        //f->glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(GLfloat), reinterpret_cast<void *>(3 * sizeof(GLfloat)));
        glDrawArrays( GL_POINTS, 0,  mPointVBO.size() / 3 / sizeof(GLfloat) );

        mPrgOutline->release();
    }
    mPointVBO.release();



}


float cGLWidget::deg2rad(int deg)
{
    return deg * 0.0174532925;
}

void cGLWidget::keyPressEvent(QKeyEvent *ke)
{
    QVector3D eyeDir = (mView-mEye).normalized();

    if(ke->key() == Qt::Key_W)
    {
        mEye += eyeDir;
        mView += eyeDir;

        qd << "eye" << mEye << "view" << mView << "up" << mUp;
    }
    if(ke->key() == Qt::Key_S)
    {
        mEye += -eyeDir;
        mView += -eyeDir;

        qd << "eye" << mEye << "view" << mView << "up" << mUp;
    }
    if(ke->key() == Qt::Key_A)
    {
        QVector3D xaxis = QVector3D::crossProduct(mUp, eyeDir).normalized();

        mEye += xaxis;
        mView += xaxis;

        qd << "eye" << mEye << "view" << mView << "up" << mUp;
    }
    if(ke->key() == Qt::Key_D)
    {
        QVector3D xaxis = QVector3D::crossProduct(mUp, eyeDir).normalized();

        mEye += -xaxis;
        mView += -xaxis;

        qd << "eye" << mEye << "view" << mView << "up" << mUp;
    }

    if(ke->key() == Qt::Key_Space)
    {
        mEye += mUp;
        mView += mUp;
    }

    update();
}

void cGLWidget::mouseMoveEvent(QMouseEvent *me)
{
    QVector3D eyeDir = (mView-mEye).normalized();

    if(mLeftMouseButtonPressed && !mSamplingMode)
    {
        QQuaternion result;

        float xangle = -deg2rad(mLeftMouseButtonPressCoordX - me->x()) * 0.1;
        float yangle = deg2rad(mLeftMouseButtonPressCoordY - me->y()) * 0.1;

        mLeftMouseButtonPressCoordX = me->x();
        mLeftMouseButtonPressCoordY = me->y();

        QVector3D yaxis = QVector3D(0,1,0); // rotate around the world's up vector
        QQuaternion r_quat(cos(xangle/2), yaxis.x()*sin(xangle/2), yaxis.y()*sin(xangle/2), yaxis.z()*sin(xangle/2));
        QQuaternion v_quat(0, eyeDir); // create view quaternion from view vector
        QQuaternion u_quat(0, mUp);
        result = (r_quat * v_quat) * r_quat.conjugate();
        QVector3D eyeShit = QVector3D(result.x(), result.y(), result.z()).normalized();
        qd << "eyeshit" << eyeShit;
        mView = eyeShit * (mView-mEye).length();

        result = (r_quat * u_quat) * r_quat.conjugate();
        mUp = QVector3D(result.x(), result.y(), result.z()).normalized();


        eyeDir = (mView-mEye).normalized();

        // rotation axis
        QVector3D xaxis = QVector3D::crossProduct(mUp, eyeDir).normalized();

        QQuaternion rr_quat(cos(xangle/2), xaxis.x()*sin(yangle/2), xaxis.y()*sin(yangle/2), xaxis.z()*sin(yangle/2));
        QQuaternion vv_quat(0, eyeDir); // create view quaternion from view vector
        QQuaternion uu_quat(0, mUp);
        result = (rr_quat * vv_quat) * rr_quat.conjugate();
        eyeShit = QVector3D(result.x(), result.y(), result.z()).normalized();
        mView = eyeShit * (mView - mEye).length();

        result = (rr_quat * uu_quat) * rr_quat.conjugate();
        mUp = QVector3D(result.x(), result.y(), result.z()).normalized();

        update();
    }

    update();
}

void cGLWidget::mousePressEvent(QMouseEvent *me)
{
    if(me->button() == Qt::LeftButton) {
        mLeftMouseButtonPressed = true;
        mLeftMouseButtonPressCoordX = me->x();
        mLeftMouseButtonPressCoordY = me->y();
    }

    update();
}

void cGLWidget::mouseReleaseEvent(QMouseEvent *me)
{
    if(me->button() == Qt::LeftButton) {
        // save total distance to apply to the new rotation at next mouseMoveEvent
        mSavedX += mLeftMouseButtonPressCoordX - me->x();
        mSavedY += mLeftMouseButtonPressCoordY - me->y();

        mLeftMouseButtonPressed = false;
    }
}

