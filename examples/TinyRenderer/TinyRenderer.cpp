#include "TinyRenderer.h"

#include <vector>
#include <limits>
#include <iostream>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"
#include "our_gl.h"
#include "../Utils/b3ResourcePath.h"
#include "Bullet3Common/b3MinMax.h"
#include "../OpenGLWindow/ShapeData.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btVector3.h"
Vec3f light_dir_world(0,0,0);


struct Shader : public IShader {
    
    Model* m_model;
    Vec3f m_light_dir_local;
    Matrix& m_modelMat;
    Matrix& m_modelView1;
    Matrix& m_projectionMatrix;
    Vec3f m_localScaling;

    mat<2,3,float> varying_uv;  // triangle uv coordinates, written by the vertex shader, read by the fragment shader
    mat<4,3,float> varying_tri; // triangle coordinates (clip coordinates), written by VS, read by FS
    mat<3,3,float> varying_nrm; // normal per vertex to be interpolated by FS
    //mat<3,3,float> ndc_tri;     // triangle in normalized device coordinates

    Shader(Model* model, Vec3f light_dir_local, Matrix& modelView, Matrix& projectionMatrix, Matrix& modelMat, Vec3f localScaling)
    :m_model(model),
    m_light_dir_local(light_dir_local),
    m_modelView1(modelView),
    m_projectionMatrix(projectionMatrix),
    m_modelMat(modelMat),
	m_localScaling(localScaling)
    {
        
    }
    
    virtual Vec4f vertex(int iface, int nthvert) {
		
		Vec2f uv = m_model->uv(iface, nthvert);
		//printf("uv = %f,%f\n", uv.x,uv.y);
        varying_uv.set_col(nthvert, uv);
		
        //varying_nrm.set_col(nthvert, proj<3>((m_projectionMatrix*m_modelView).invert_transpose()*embed<4>(m_model->normal(iface, nthvert), 0.f)));
        varying_nrm.set_col(nthvert, proj<3>((m_modelMat).invert_transpose()*embed<4>(m_model->normal(iface, nthvert), 0.f)));
		Vec3f unScaledVert = m_model->vert(iface, nthvert);
		Vec3f scaledVert=Vec3f(unScaledVert[0]*m_localScaling[0],unScaledVert[1]*m_localScaling[1],unScaledVert[2]*m_localScaling[2]);
        Vec4f gl_Vertex = m_projectionMatrix*m_modelView1*embed<4>(scaledVert);
		
        varying_tri.set_col(nthvert, gl_Vertex);
        //ndc_tri.set_col(nthvert, proj<3>(gl_Vertex/gl_Vertex[3]));
        return gl_Vertex;
    }

    virtual bool fragment(Vec3f bar, TGAColor &color) {
        Vec3f bn = (varying_nrm*bar).normalize();
        Vec2f uv = varying_uv*bar;

		//float diff = 1;//full-bright
		float ambient = 0.7;
		float diff = ambient+b3Min(b3Max(0.f, bn*light_dir_world),(1-ambient));
		//float diff = b3Max(0.f, n*m_light_dir_local);
        color = m_model->diffuse(uv)*diff;

        return false;
    }
};


TinyRenderObjectData::TinyRenderObjectData(int width, int height,TGAImage& rgbColorBuffer,b3AlignedObjectArray<float>&depthBuffer)
:m_width(width),
m_height(height),
m_rgbColorBuffer(rgbColorBuffer),
m_depthBuffer(depthBuffer),
m_model(0),
m_userData(0),
m_userIndex(-1)
{
    Vec3f       eye(1,1,3);
    Vec3f    center(0,0,0);
    Vec3f        up(0,0,1);
    m_lightDirWorld.setValue(0,0,0);
	m_localScaling.setValue(1,1,1);
    m_modelMatrix = Matrix::identity();
    m_viewMatrix = lookat(eye, center, up);
    //m_viewportMatrix = viewport(width/8, height/8, width*3/4, height*3/4);
	//m_viewportMatrix = viewport(width/8, height/8, width*3/4, height*3/4);
	m_viewportMatrix = viewport(0,0,width,height);
    m_projectionMatrix = projection(-1.f/(eye-center).norm());
 
}

void TinyRenderObjectData::loadModel(const char* fileName)
{
 //todo(erwincoumans) move the file loading out of here
   char relativeFileName[1024];
    if (!b3ResourcePath::findResourcePath(fileName, relativeFileName, 1024))
    {
        printf("Cannot find file %s\n", fileName);
    } else
    {
        m_model = new Model(relativeFileName);
    }   
}


void TinyRenderObjectData::registerMeshShape(const float* vertices, int numVertices,const int* indices, int numIndices,
	unsigned char* textureImage, int textureWidth, int textureHeight)
{
	if (0==m_model)
    {
        m_model = new Model();
		if (textureImage)
		{
			m_model->setDiffuseTextureFromData(textureImage,textureWidth,textureHeight);
		} else
		{
			char relativeFileName[1024];
			if (b3ResourcePath::findResourcePath("floor_diffuse.tga", relativeFileName, 1024))
			{
				m_model->loadDiffuseTexture(relativeFileName);
			}
		}
		
        for (int i=0;i<numVertices;i++)
        {
            m_model->addVertex(vertices[i*9],
                         vertices[i*9+1],
                         vertices[i*9+2],
                         vertices[i*9+4],
                         vertices[i*9+5],
                         vertices[i*9+6],
                         vertices[i*9+7],
                         vertices[i*9+8]);
        }
        for (int i=0;i<numIndices;i+=3)
        {
            m_model->addTriangle(indices[i],indices[i],indices[i],
                                 indices[i+1],indices[i+1],indices[i+1],
                                 indices[i+2],indices[i+2],indices[i+2]);
        }
    }
}

void TinyRenderObjectData::registerMesh2(btAlignedObjectArray<btVector3>& vertices, btAlignedObjectArray<btVector3>& normals,btAlignedObjectArray<int>& indices)
{
	if (0==m_model)
    {
		int numVertices = vertices.size();
		int numIndices = indices.size();

        m_model = new Model();
		char relativeFileName[1024];
		if (b3ResourcePath::findResourcePath("floor_diffuse.tga", relativeFileName, 1024))
		{
			m_model->loadDiffuseTexture(relativeFileName);
		}
		
        for (int i=0;i<numVertices;i++)
        {
            m_model->addVertex(vertices[i].x(),
                         vertices[i].y(),
                         vertices[i].z(),
                         normals[i].x(),
                         normals[i].y(),
                         normals[i].z(),
                         0.5,0.5);
        }
        for (int i=0;i<numIndices;i+=3)
        {
            m_model->addTriangle(indices[i],indices[i],indices[i],
                                 indices[i+1],indices[i+1],indices[i+1],
                                 indices[i+2],indices[i+2],indices[i+2]);
        }
    }
}

void TinyRenderObjectData::createCube(float halfExtentsX,float halfExtentsY,float halfExtentsZ)
{
    m_model = new Model();
    
    char relativeFileName[1024];
    if (b3ResourcePath::findResourcePath("floor_diffuse.tga", relativeFileName, 1024))
    {
        m_model->loadDiffuseTexture(relativeFileName);
    }
    

	int strideInBytes = 9*sizeof(float);
	int numVertices = sizeof(cube_vertices_textured)/strideInBytes;
	int numIndices = sizeof(cube_indices)/sizeof(int);

	for (int i=0;i<numVertices;i++)
	{
		m_model->addVertex(halfExtentsX*cube_vertices_textured[i*9],
                     halfExtentsY*cube_vertices_textured[i*9+1],
                     halfExtentsY*cube_vertices_textured[i*9+2],
                     cube_vertices_textured[i*9+4],
                     cube_vertices_textured[i*9+5],
                     cube_vertices_textured[i*9+6],
                     cube_vertices_textured[i*9+7],
                     cube_vertices_textured[i*9+8]);
	}
	for (int i=0;i<numIndices;i+=3)
    {
        m_model->addTriangle(cube_indices[i],cube_indices[i],cube_indices[i],
                             cube_indices[i+1],cube_indices[i+1],cube_indices[i+1],
                             cube_indices[i+2],cube_indices[i+2],cube_indices[i+2]);
    }
	
}

TinyRenderObjectData::~TinyRenderObjectData()
{
    delete m_model;
}

void TinyRenderer::renderObject(TinyRenderObjectData& renderData)
{
	light_dir_world = Vec3f(renderData.m_lightDirWorld[0],renderData.m_lightDirWorld[1],renderData.m_lightDirWorld[2]);
    Model* model = renderData.m_model;
    if (0==model)
        return;
    
	

	//renderData.m_viewMatrix = lookat(eye, center, up);
	int width = renderData.m_width;
	int height = renderData.m_height;
	//renderData.m_viewportMatrix = viewport(width/8, height/8, width*3/4, height*3/4);
	renderData.m_viewportMatrix = viewport(0,0,renderData.m_width,renderData.m_height);
    //renderData.m_projectionMatrix = projection(-1.f/(eye-center).norm());
 


    b3AlignedObjectArray<float>& zbuffer = renderData.m_depthBuffer;
    
    TGAImage& frame = renderData.m_rgbColorBuffer;

	Vec3f light_dir_local = proj<3>((renderData.m_projectionMatrix*renderData.m_viewMatrix*renderData.m_modelMatrix*embed<4>(light_dir_world, 0.f))).normalize();

    {
        Matrix modelViewMatrix = renderData.m_viewMatrix*renderData.m_modelMatrix;
        Vec3f localScaling(renderData.m_localScaling[0],renderData.m_localScaling[1],renderData.m_localScaling[2]);
        Shader shader(model, light_dir_local, modelViewMatrix, renderData.m_projectionMatrix,renderData.m_modelMatrix, localScaling);
        
		
		
		for (int i=0; i<model->nfaces(); i++) {
			 
            for (int j=0; j<3; j++) {
                shader.vertex(i, j);
            }
            triangle(shader.varying_tri, shader, frame, &zbuffer[0], renderData.m_viewportMatrix);
        }
        
    }
        
}


