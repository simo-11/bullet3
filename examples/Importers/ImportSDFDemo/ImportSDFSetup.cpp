
#include "ImportSDFSetup.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"


#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "Bullet3Common/b3FileUtils.h"

#include "BulletDynamics/Featherstone/btMultiBodyJointMotor.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "../CommonInterfaces/CommonParameterInterface.h"
#include "../../Utils/b3ResourcePath.h"

#include "../ImportURDFDemo/BulletUrdfImporter.h"
#include "../Importers/ImportURDFDemo/DefaultVisualShapeConverter.h"

#include "../ImportURDFDemo/URDF2Bullet.h"


//#include "urdf_samples.h"





#include "../CommonInterfaces/CommonMultiBodyBase.h"

#include "../ImportURDFDemo/MyMultiBodyCreator.h"


class ImportSDFSetup : public CommonMultiBodyBase
{
    char m_fileName[1024];

    struct ImportSDFInternalData* m_data;
	bool m_useMultiBody;

    //todo(erwincoumans) we need a name memory for each model
	btAlignedObjectArray<std::string* > m_nameMemory;

public:
    ImportSDFSetup(struct GUIHelperInterface* helper, int option, const char* fileName);
    virtual ~ImportSDFSetup();

	virtual void initPhysics();
	virtual void stepSimulation(float deltaTime);

    void setFileName(const char* urdfFileName);

	virtual void resetCamera()
	{
		float dist = 3.5;
		float pitch = -136;
		float yaw = 28;
		float targetPos[3]={0.47,0,-0.64};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}
};


static btAlignedObjectArray<std::string> gFileNameArray;


#define MAX_NUM_MOTORS 1024

struct ImportSDFInternalData
{
    ImportSDFInternalData()
    :m_numMotors(0)
    {
		for (int i=0;i<MAX_NUM_MOTORS;i++)
		{
			m_jointMotors[i] = 0;
			m_generic6DofJointMotors[i] = 0;
		}
    }

    btScalar m_motorTargetVelocities[MAX_NUM_MOTORS];
    btMultiBodyJointMotor* m_jointMotors [MAX_NUM_MOTORS];
	btGeneric6DofSpring2Constraint* m_generic6DofJointMotors [MAX_NUM_MOTORS];
    int m_numMotors;

};


ImportSDFSetup::ImportSDFSetup(struct GUIHelperInterface* helper, int option, const char* fileName)
	:CommonMultiBodyBase(helper)
{
	m_data = new ImportSDFInternalData;
	(void)option;

//	if (option==1)
//	{
		m_useMultiBody = true;
//
//	} else
//	{
//		m_useMultiBody = false;
//	}

	static int count = 0;
	if (fileName)
	{
		setFileName(fileName);
	} else
	{
		gFileNameArray.clear();
		


		//load additional urdf file names from file

		FILE* f = fopen("sdf_files.txt","r");
		if (f)
		{
			int result;
			//warning: we don't avoid string buffer overflow in this basic example in fscanf
			char fileName[1024];
			do
			{
				result = fscanf(f,"%s",fileName);
                b3Printf("sdf_files.txt entry %s",fileName);
				if (result==1)
				{
					gFileNameArray.push_back(fileName);
				}
			} while (result==1);

			fclose(f);
		}
		
		if (gFileNameArray.size()==0)
		{
			gFileNameArray.push_back("two_cubes.sdf");

		}

		int numFileNames = gFileNameArray.size();

		if (count>=numFileNames)
		{
			count=0;
		}
		sprintf(m_fileName,"%s",gFileNameArray[count++].c_str());
	}
}

ImportSDFSetup::~ImportSDFSetup()
{
	for (int i=0;i<m_nameMemory.size();i++)
	{
		delete m_nameMemory[i];
	}
	m_nameMemory.clear();
    delete m_data;
}

static btVector4 colors[4] =
{
	btVector4(1,0,0,1),
	btVector4(0,1,0,1),
	btVector4(0,1,1,1),
	btVector4(1,1,0,1),
};


static btVector3 selectColor()
{

	static int curColor = 0;
	btVector4 color = colors[curColor];
	curColor++;
	curColor&=3;
	return color;
}

void ImportSDFSetup::setFileName(const char* urdfFileName)
{
    memcpy(m_fileName,urdfFileName,strlen(urdfFileName)+1);
}




void ImportSDFSetup::initPhysics()
{

	int upAxis = 2;
	m_guiHelper->setUpAxis(upAxis);

	this->createEmptyDynamicsWorld();
	//m_dynamicsWorld->getSolverInfo().m_numIterations = 100;
    m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
    m_dynamicsWorld->getDebugDrawer()->setDebugMode(
    btIDebugDraw::DBG_DrawConstraints
    +btIDebugDraw::DBG_DrawContactPoints
    +btIDebugDraw::DBG_DrawAabb
        );//+btIDebugDraw::DBG_DrawConstraintLimits);


	btVector3 gravity(0,0,0);
	gravity[upAxis]=-9.8;

	m_dynamicsWorld->setGravity(gravity);

	DefaultVisualShapeConverter visualConverter(m_guiHelper);
    BulletURDFImporter u2b(m_guiHelper, &visualConverter);
    
    bool loadOk =  u2b.loadSDF(m_fileName);


	if (loadOk)
	{
		//printTree(u2b,u2b.getRootLinkIndex());

		//u2b.printTree();

        btTransform rootTrans;
        rootTrans.setIdentity();
        
        for (int m =0; m<u2b.getNumModels();m++)
		{

            u2b.activateModel(m);
            
			btMultiBody* mb = 0;


			//todo: move these internal API called inside the 'ConvertURDF2Bullet' call, hidden from the user
			int rootLinkIndex = u2b.getRootLinkIndex();
			b3Printf("urdf root link index = %d\n",rootLinkIndex);
			MyMultiBodyCreator creation(m_guiHelper);

            u2b.getRootTransformInWorld(rootTrans);
			ConvertURDF2Bullet(u2b,creation, rootTrans,m_dynamicsWorld,m_useMultiBody,u2b.getPathPrefix(),true);
			mb = creation.getBulletMultiBody();
			
		}

	
        for (int i=0;i<m_dynamicsWorld->getNumMultiBodyConstraints();i++)
        {
            m_dynamicsWorld->getMultiBodyConstraint(i)->finalizeMultiDof();
        }



		bool createGround=true;
		if (createGround)
		{
			btVector3 groundHalfExtents(20,20,20);
			groundHalfExtents[upAxis]=1.f;
			btBoxShape* box = new btBoxShape(groundHalfExtents);
			box->initializePolyhedralFeatures();

			m_guiHelper->createCollisionShapeGraphicsObject(box);
			btTransform start; start.setIdentity();
			btVector3 groundOrigin(0,0,0);
			groundOrigin[upAxis]=-2.5;
			start.setOrigin(groundOrigin);
			btRigidBody* body =  createRigidBody(0,start,box);
			//m_dynamicsWorld->removeRigidBody(body);
		   // m_dynamicsWorld->addRigidBody(body,2,1);
			btVector3 color(0.5,0.5,0.5);
			m_guiHelper->createRigidBodyGraphicsObject(body,color);
		}

		///this extra stepSimulation call makes sure that all the btMultibody transforms are properly propagates.
		m_dynamicsWorld->stepSimulation(1. / 240., 0);// 1., 10, 1. / 240.);
	}


}

void ImportSDFSetup::stepSimulation(float deltaTime)
{
	if (m_dynamicsWorld)
	{
        for (int i=0;i<m_data->m_numMotors;i++)
        {
			if (m_data->m_jointMotors[i])
			{
				m_data->m_jointMotors[i]->setVelocityTarget(m_data->m_motorTargetVelocities[i]);
			}
			if (m_data->m_generic6DofJointMotors[i])
			{
				GenericConstraintUserInfo* jointInfo = (GenericConstraintUserInfo*)m_data->m_generic6DofJointMotors[i]->getUserConstraintPtr();
				m_data->m_generic6DofJointMotors[i]->setTargetVelocity(jointInfo->m_jointAxisIndex,m_data->m_motorTargetVelocities[i]);
				//jointInfo->
			}
        }

		//the maximal coordinates/iterative MLCP solver requires a smallish timestep to converge
		m_dynamicsWorld->stepSimulation(deltaTime,10,1./240.);
	}
}

class CommonExampleInterface*    ImportSDFCreateFunc(struct CommonExampleOptions& options)
{
	
	return new ImportSDFSetup(options.m_guiHelper, options.m_option,options.m_fileName);
}
