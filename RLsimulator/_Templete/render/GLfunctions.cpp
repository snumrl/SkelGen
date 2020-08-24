#include "GLfunctions.h"
#include <assimp/cimport.h>
#include <iostream>
#include <GL/gl.h>
#include <GL/glu.h>
#include "GL/glut.h"

static GLUquadricObj *quadObj;
static void initQuadObj(void)
{
    quadObj = gluNewQuadric();
    if(!quadObj)
        // DART modified error output
        std::cerr << "OpenGL: Fatal Error in DART: out of memory." << std::endl;
}

#define QUAD_OBJ_INIT { if(!quadObj) initQuadObj(); }
void
GUI::
DrawSphere(double r)
{
    QUAD_OBJ_INIT;
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    gluQuadricDrawStyle(quadObj, GLU_FILL);
    gluQuadricNormals(quadObj, GLU_SMOOTH);

    gluSphere(quadObj, r, 8, 8);
    glLineWidth(1.0);
    float oc[3];
    glGetFloatv(GL_CURRENT_COLOR,oc);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    gluQuadricDrawStyle(quadObj, GLU_LINE);
    glColor3f(0,0,0);
    // gluSphere(quadObj, r+0.0005, 6, 6);
    glColor3f(oc[0],oc[1],oc[2]);
}
void
GUI::
DrawSphere(const Eigen::Vector3d& pos,double r)
{
    glPushMatrix();
    glTranslatef(pos[0],pos[1],pos[2]);
    DrawSphere(r);
    glPopMatrix();
}
void
GUI::
DrawCube(const Eigen::Vector3d& _size)
{
    glScaled(_size(0), _size(1), _size(2));

    // Code taken from glut/lib/glut_shapes.c
    static GLfloat n[6][3] =
    {
        {-1.0, 0.0, 0.0},
        {0.0, 1.0, 0.0},
        {1.0, 0.0, 0.0},
        {0.0, -1.0, 0.0},
        {0.0, 0.0, 1.0},
        {0.0, 0.0, -1.0}
    };
    static GLint faces[6][4] =
    {
        {0, 1, 2, 3},
        {3, 2, 6, 7},
        {7, 6, 5, 4},
        {4, 5, 1, 0},
        {5, 6, 2, 1},
        {7, 4, 0, 3}
    };
    GLfloat v[8][3];
    GLint i;
    GLfloat size = 1;

    v[0][0] = v[1][0] = v[2][0] = v[3][0] = -size / 2;
    v[4][0] = v[5][0] = v[6][0] = v[7][0] = size / 2;
    v[0][1] = v[1][1] = v[4][1] = v[5][1] = -size / 2;
    v[2][1] = v[3][1] = v[6][1] = v[7][1] = size / 2;
    v[0][2] = v[3][2] = v[4][2] = v[7][2] = -size / 2;
    v[1][2] = v[2][2] = v[5][2] = v[6][2] = size / 2;

    glPolygonMode(GL_FRONT, GL_FILL);
    for (i = 5; i >= 0; i--) {
        glBegin(GL_QUADS);
        glNormal3fv(&n[i][0]);
        glVertex3fv(&v[faces[i][0]][0]);
        glVertex3fv(&v[faces[i][1]][0]);
        glVertex3fv(&v[faces[i][2]][0]);
        glVertex3fv(&v[faces[i][3]][0]);
        glEnd();
        
        
        
    }
    glLineWidth(2.0);
    float oc[3];
    glGetFloatv(GL_CURRENT_COLOR,oc);
    glColor3f(0,0,0);
    glPolygonMode(GL_FRONT, GL_LINE);
    for (i = 5; i >= 0; i--) {
        glBegin(GL_QUADS);
        glNormal3fv(&n[i][0]);
        glVertex3fv(&v[faces[i][0]][0]);
        glVertex3fv(&v[faces[i][1]][0]);
        glVertex3fv(&v[faces[i][2]][0]);
        glVertex3fv(&v[faces[i][3]][0]);
        glEnd();
    }
    glColor3f(oc[0],oc[1],oc[2]);
}
void
GUI::
DrawTetrahedron(const Eigen::Vector3d& p0,const Eigen::Vector3d& p1,const Eigen::Vector3d& p2,const Eigen::Vector3d& p3,const Eigen::Vector3d& color)
{
	DrawTriangle(p0,p1,p2,color);
	DrawTriangle(p0,p1,p3,color);
	DrawTriangle(p0,p2,p3,color);
	DrawTriangle(p1,p2,p3,color);
}
void
GUI::
DrawTriangle(const Eigen::Vector3d& p0,const Eigen::Vector3d& p1,const Eigen::Vector3d& p2,const Eigen::Vector3d& color)
{
	glColor3f(color[0],color[1],color[2]);
	glBegin(GL_TRIANGLES);
	glVertex3f(p0[0],p0[1],p0[2]);
	glVertex3f(p1[0],p1[1],p1[2]);
	glVertex3f(p2[0],p2[1],p2[2]);
	glEnd();
}
void
GUI::
DrawLine(const Eigen::Vector3d& p0,const Eigen::Vector3d& p1,const Eigen::Vector3d& color)
{
	glColor3f(color[0],color[1],color[2]);
	glBegin(GL_LINES);
	glVertex3f(p0[0],p0[1],p0[2]);
	glVertex3f(p1[0],p1[1],p1[2]);
	glEnd();
}
void
GUI::
DrawPoint(const Eigen::Vector3d& p0,const Eigen::Vector3d& color)
{
	glColor3f(color[0],color[1],color[2]);
	glBegin(GL_POINTS);
	glVertex3f(p0[0],p0[1],p0[2]);
	glEnd();
}
void
GUI::
DrawArrow3D(const Eigen::Vector3d& _pt, const Eigen::Vector3d& _dir,
            const double _length, const double _thickness,const Eigen::Vector3d& color,
            const double _arrowThickness)
{
    glColor3f(color[0],color[1],color[2]);
    Eigen::Vector3d normDir = _dir;
  normDir.normalize();

  double arrowLength;
  if (_arrowThickness == -1)
    arrowLength = 4*_thickness;
  else
    arrowLength = 2*_arrowThickness;

  // draw the arrow body as a cylinder
  GLUquadricObj *c;
  c = gluNewQuadric();
  gluQuadricDrawStyle(c, GLU_FILL);
  gluQuadricNormals(c, GLU_SMOOTH);

  glPushMatrix();
  glTranslatef(_pt[0], _pt[1], _pt[2]);
  glRotated(acos(normDir[2])*180/M_PI, -normDir[1], normDir[0], 0);
  gluCylinder(c, _thickness, _thickness, _length-arrowLength, 16, 16);

  // draw the arrowhed as a cone
  glPushMatrix();
  glTranslatef(0, 0, _length-arrowLength);
  gluCylinder(c, arrowLength*0.5, 0.0, arrowLength, 10, 10);
  glPopMatrix();

  glPopMatrix();

  gluDeleteQuadric(c);
}

void recursiveRender(const struct aiScene *sc, const struct aiNode* nd) {
    unsigned int i;
    unsigned int n = 0, t;
    aiMatrix4x4 m = nd->mTransformation;
    float cc[4];
    glGetFloatv(GL_CURRENT_COLOR,cc);
    // update transform
    aiTransposeMatrix4(&m);
    glPushMatrix();
    glMultMatrixf((float*)&m);

    // draw all meshes assigned to this node
    for (; n < nd->mNumMeshes; ++n) {
        const struct aiMesh* mesh = sc->mMeshes[nd->mMeshes[n]];

        glPushAttrib(GL_POLYGON_BIT | GL_LIGHTING_BIT);  // for applyMaterial()

        if(mesh->mNormals == nullptr) { glDisable(GL_LIGHTING);
        } else {
            glEnable(GL_LIGHTING);
        }
        
        // const aiMaterial *mtl = sc->mMaterials[mesh->mMaterialIndex];

        // mesh->mColors[0] = cc[0];
        // mesh->mColors[1] = cc[1];
        // mesh->mColors[2] = cc[2];
        // mesh->mColors[3] = cc[3];
        for (t = 0; t < mesh->mNumFaces; ++t) {
            const struct aiFace* face = &mesh->mFaces[t];
            GLenum face_mode;

            switch(face->mNumIndices) {
                case 1: face_mode = GL_POINTS; break;
                case 2: face_mode = GL_LINES; break;
                case 3: face_mode = GL_TRIANGLES; break;
                default: face_mode = GL_POLYGON; break;
            }

            glBegin(face_mode);

            for (i = 0; i < face->mNumIndices; i++) {
                int index = face->mIndices[i];
                // if(mesh->mColors[0] != nullptr)
                //     glColor4fv((GLfloat*)&mesh->mColors[0][index]);
                  // glColor4f(1.0,1.0,1.0,1.0);
                if(mesh->mNormals != nullptr)
                    glNormal3fv(&mesh->mNormals[index].x);
                glVertex3fv(&mesh->mVertices[index].x);
            }

            glEnd();
        }

        glPopAttrib();  // for applyMaterial()
    }

    // draw all children
    for (n = 0; n < nd->mNumChildren; ++n) {
        recursiveRender(sc, nd->mChildren[n]);
    }

    glPopMatrix();
}
void GUI::drawCylinder(double r, Eigen::Vector3d p,Eigen::Vector3d p1)
{
    Eigen::Vector3d u(0,0,1);
    Eigen::Vector3d v = p-p1;
    Eigen::Vector3d mid = 0.5*(p+p1);
    double len = v.norm();
    v /= len;
    Eigen::Isometry3d T;
    T.setIdentity();
    Eigen::Vector3d axis = u.cross(v);
    axis.normalize();
    double angle = acos(u.dot(v));
    Eigen::Matrix3d w_bracket = Eigen::Matrix3d::Zero();
    w_bracket(0, 1) = -axis(2);
    w_bracket(1, 0) =  axis(2);
    w_bracket(0, 2) =  axis(1);
    w_bracket(2, 0) = -axis(1);
    w_bracket(1, 2) = -axis(0);
    w_bracket(2, 1) =  axis(0);

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity()+(sin(angle))*w_bracket+(1.0-cos(angle))*w_bracket*w_bracket;
    // std::cout<<R.determinant()<<std::endl;
    T.linear() = R;
    T.translation() = mid;
    glPushMatrix();
    glMultMatrixd(T.data());
    // GUI::DrawSphere(0.03);
    GUI::drawCylinder(r,len,6,6);
    glPopMatrix();
}
void GUI::drawCylinder(
    double radius, double height, int slices, int stacks)
{
  glPushMatrix();

  // Graphics assumes Cylinder is centered at CoM
  // gluCylinder places base at z = 0 and top at z = height
  glTranslated(0.0, 0.0, -0.5*height);

  // Code taken from glut/lib/glut_shapes.c
  QUAD_OBJ_INIT;
  gluQuadricDrawStyle(quadObj, GLU_FILL);
  gluQuadricNormals(quadObj, GLU_SMOOTH);
  //gluQuadricTexture(quadObj, GL_TRUE);

  // glut/lib/glut_shapes.c
  gluCylinder(quadObj, radius, radius, height, slices, stacks);

  glPopMatrix();

  glPushMatrix();

  glScaled(radius, radius, height);

  QUAD_OBJ_INIT;
  gluQuadricDrawStyle(quadObj, GLU_FILL);
  gluQuadricNormals(quadObj, GLU_SMOOTH);
  //gluQuadricTexture(quadObj, GL_TRUE);

  glTranslated(0.0, 0.0, 0.5);
  gluDisk(quadObj, 0, 1, slices, stacks);
  glTranslated(0.0, 0.0, -1.0);
  gluDisk(quadObj, 0, 1, slices, stacks);

  glPopMatrix();
}
void
GUI::
DrawBezierCurve(
    const Eigen::Vector3d& p0,
    const Eigen::Vector3d& p1,
    const Eigen::Vector3d& p2,
    const Eigen::Vector3d& color)
{
    glColor3f(color[0],color[1],color[2]);
    glBegin(GL_LINE_STRIP);
    for(double s = 0;s<=1.0;s+=0.05)
    {
        Eigen::Vector3d p = 
            p0*(1-s)*(1-s)+
            p1*2*s*(1-s)+
            p2*s*s;

        glVertex3f(p[0],p[1],p[2]);
    }
    glEnd();
}

void
GUI::
DrawMesh(const Eigen::Vector3d& scale, const aiScene* mesh,const Eigen::Vector3d& color)
{
 if (!mesh)
    return;
  // glColor3f(color[0],color[1],color[2]);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  // glColor3f(1.0,1.0,1.0);
  glPushMatrix();

  glScaled(scale[0], scale[1], scale[2]);
  recursiveRender(mesh, mesh->mRootNode);

  glPopMatrix();
}


void
GUI::
DrawStringOnScreen(float _x, float _y, const std::string& _s,bool _bigFont,const Eigen::Vector3d& color)
{
    glColor3f(color[0],color[1],color[2]);
	
    // draws text on the screen
    GLint oldMode;
    glGetIntegerv(GL_MATRIX_MODE, &oldMode);
    glMatrixMode(GL_PROJECTION);

    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0.0, 1.0, 0.0, 1.0);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glRasterPos2f(_x, _y);
    unsigned int length = _s.length();
    for (unsigned int c = 0; c < length; c++) {
    if (_bigFont)
      glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, _s.at(c) );
    else
      glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, _s.at(c) );
    }  
    glPopMatrix();

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(oldMode);
}

