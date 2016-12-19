#ifndef TINY_RENDERER_H
#define TINY_RENDERER_H

#include "geometry.h"
#include "Bullet3Common/b3AlignedObjectArray.h"
#include "Bullet3Common/b3Vector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btVector3.h"


#include "tgaimage.h"

struct TinyRenderObjectData
{
    //Camera
    Matrix m_viewMatrix;
    Matrix m_projectionMatrix;
    Matrix m_viewportMatrix;
	btVector3 m_localScaling;
	btVector3 m_lightDirWorld;
    btVector3 m_lightColor;
    float m_lightDistance;
    float m_lightAmbientCoeff;
    float m_lightDiffuseCoeff;
    float m_lightSpecularCoeff;
	
    //Model (vertices, indices, textures, shader)
    Matrix m_modelMatrix;
    class Model*  m_model;
    //class IShader* m_shader; todo(erwincoumans) expose the shader, for now we use a default shader
            
    //Output
    
    TGAImage& m_rgbColorBuffer;
    b3AlignedObjectArray<float>& m_depthBuffer;//required, hence a reference
    b3AlignedObjectArray<float>* m_shadowBuffer;//optional, hence a pointer
    b3AlignedObjectArray<int>* m_segmentationMaskBufferPtr;//optional, hence a pointer
    
    TinyRenderObjectData(TGAImage& rgbColorBuffer,b3AlignedObjectArray<float>&depthBuffer);
    TinyRenderObjectData(TGAImage& rgbColorBuffer,b3AlignedObjectArray<float>&depthBuffer,b3AlignedObjectArray<int>* segmentationMaskBuffer,int objectIndex);
    TinyRenderObjectData(TGAImage& rgbColorBuffer,b3AlignedObjectArray<float>&depthBuffer,b3AlignedObjectArray<float>* shadowBuffer);
    TinyRenderObjectData(TGAImage& rgbColorBuffer,b3AlignedObjectArray<float>&depthBuffer,b3AlignedObjectArray<float>* shadowBuffer, b3AlignedObjectArray<int>* segmentationMaskBuffer,int objectIndex);
    virtual ~TinyRenderObjectData();
    
    void loadModel(const char* fileName);
    void createCube(float HalfExtentsX,float HalfExtentsY,float HalfExtentsZ);
    void registerMeshShape(const float* vertices, int numVertices,const int* indices, int numIndices, const float rgbaColor[4],
		unsigned char* textureImage=0, int textureWidth=0, int textureHeight=0);
	
	void registerMesh2(btAlignedObjectArray<btVector3>& vertices, btAlignedObjectArray<btVector3>& normals,btAlignedObjectArray<int>& indices);
    
    void* m_userData;
    int m_userIndex;
    int m_objectIndex;
};


class TinyRenderer
{
    public:
        static void renderObjectDepth(TinyRenderObjectData& renderData);
        static void renderObject(TinyRenderObjectData& renderData);
};

#endif // TINY_RENDERER_Hbla
