
#include "PhysicsClientExample.h"

#include "../CommonInterfaces/CommonMultiBodyBase.h"
#include "../CommonInterfaces/Common2dCanvasInterface.h"
#include "SharedMemoryCommon.h"
#include "../CommonInterfaces/CommonParameterInterface.h"
#include "PhysicsClientC_API.h"
#include "PhysicsClient.h"
//#include "SharedMemoryCommands.h"
#include "PhysicsLoopBackC_API.h"
#include "PhysicsDirectC_API.h"
#include "PhysicsClientC_API.h"
#include "PhysicsServerSharedMemory.h"
struct MyMotorInfo2
{
	btScalar m_velTarget;
	btScalar m_maxForce;
    btScalar m_posTarget;
	int		m_uIndex;
    int     m_qIndex;
};

int camVisualizerWidth = 320;//1024/3;
int camVisualizerHeight = 240;//768/3;


#define MAX_NUM_MOTORS 128

class PhysicsClientExample : public SharedMemoryCommon
{
protected:
    b3PhysicsClientHandle m_physicsClientHandle;

	//this m_physicsServer is only used when option eCLIENTEXAMPLE_SERVER is enabled
	PhysicsServerSharedMemory	m_physicsServer;
	
	bool m_wantsTermination;
    btAlignedObjectArray<int> m_userCommandRequests;
    int m_sharedMemoryKey;
    int m_selectedBody;
	int m_prevSelectedBody;
	struct Common2dCanvasInterface* m_canvas;
	int m_canvasIndex;
	
	void	createButton(const char* name, int id, bool isTrigger );

	void createButtons();
	

    
    //@todo, add accessor methods
	// MyMotorInfo2 m_motorTargetVelocities[MAX_NUM_MOTORS];
    MyMotorInfo2 m_motorTargetPositions[MAX_NUM_MOTORS];
	int m_numMotors;
	int m_options;
	bool m_isOptionalServerConnected;

	public:

	PhysicsClientExample(GUIHelperInterface* helper, int options);
	virtual ~PhysicsClientExample();
    
	virtual void	initPhysics();
    void selectComboBox(int comboIndex, const char* name)
	{
		if (m_guiHelper && m_guiHelper->getParameterInterface())
		{
			int bodyIndex = comboIndex;
			if (m_selectedBody != bodyIndex)
			{
				m_selectedBody = bodyIndex;
			}
		}
	}
	virtual void	stepSimulation(float deltaTime);
    
	virtual void resetCamera()
	{
		float dist = 1.1;
		float pitch = 50;
		float yaw = 35;
		float targetPos[3]={0,0,0.5};//-3,2.8,-2.5};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);

	}
    
    virtual bool wantsTermination()
    {
        return m_wantsTermination;
    }
    
    virtual bool isConnected()
    {
        return (m_physicsClientHandle!=0);
    }
    
    
    
    void enqueueCommand(int commandId);
    
    void prepareAndSubmitCommand(int commandId);
    
    
	virtual void    exitPhysics(){};
	virtual void	renderScene()
	{
		if (m_options == eCLIENTEXAMPLE_SERVER)
		{
			m_physicsServer.renderScene();
		}

        b3DebugLines debugLines;
        b3GetDebugLines(m_physicsClientHandle,&debugLines);
        int numLines = debugLines.m_numDebugLines;

		int lineWidth = 1;

		if (1)
		{
			btAlignedObjectArray<btVector3FloatData> points;
			points.resize(numLines*2);
			btAlignedObjectArray<unsigned int> indices;
			indices.resize(numLines*2);

			for (int i=0;i<numLines;i++)
			{
				points[i*2].m_floats[0] = debugLines.m_linesFrom[i*3+0];
				points[i*2].m_floats[1] = debugLines.m_linesFrom[i*3+1];
                points[i*2].m_floats[2] = debugLines.m_linesFrom[i*3+2];
                points[i*2+1].m_floats[0] = debugLines.m_linesTo[i*3+0];
				points[i*2+1].m_floats[1] = debugLines.m_linesTo[i*3+1];
				points[i*2+1].m_floats[2] = debugLines.m_linesTo[i*3+2];
				indices[i*2] = i*2;
				indices[i*2+1] = i*2+1;
			}

		
			
			float color[4] = {0.2,0.2,1,1};
			
			if (points.size() && indices.size())
			{
				m_guiHelper->getRenderInterface()->drawLines(&points[0].m_floats[0],color,points.size(),sizeof(btVector3FloatData),&indices[0],indices.size(),lineWidth);
			}
		} else
		{
			for (int i=0;i<numLines;i++)
			{
				m_guiHelper->getRenderInterface()->drawLine(debugLines.m_linesFrom,debugLines.m_linesTo,debugLines.m_linesColor,lineWidth);
			}
		}
	}

	void prepareControlCommand(b3SharedMemoryCommandHandle commandHandle)
	{
        for (int i=0;i<m_numMotors;i++)
        {
            
            btScalar targetPos = m_motorTargetPositions[i].m_posTarget;
            int qIndex = m_motorTargetPositions[i].m_qIndex;
            int uIndex = m_motorTargetPositions[i].m_uIndex;
            b3JointControlSetDesiredPosition(commandHandle, qIndex, targetPos);
            b3JointControlSetKp(commandHandle, uIndex, 0.1);
            
            b3JointControlSetMaximumForce(commandHandle,uIndex,1000);
        }
	}
	virtual void	physicsDebugDraw(int debugFlags)
	{
		if (m_options==eCLIENTEXAMPLE_SERVER)
		{
			m_physicsServer.physicsDebugDraw(debugFlags);
		}
	}
	virtual bool	mouseMoveCallback(float x,float y){return false;};
	virtual bool	mouseButtonCallback(int button, int state, float x, float y){return false;}
	virtual bool	keyboardCallback(int key, int state){return false;}

	
	virtual void setSharedMemoryKey(int key)
	{
        m_sharedMemoryKey = key;
	}
};


void MyComboBoxCallback (int combobox, const char* item, void* userPointer)
{
	b3Printf("Item selected %s", item);

	PhysicsClientExample* cl = (PhysicsClientExample*) userPointer;
	b3Assert(cl);
	if (cl)
	{
		cl->selectComboBox(combobox,item);
	}
	
}



void MyCallback(int buttonId, bool buttonState, void* userPtr)
{
	PhysicsClientExample* cl = (PhysicsClientExample*) userPtr;
	b3Assert(cl);

    if (cl && buttonState)
    {
        cl->enqueueCommand(buttonId);
    }
}

void PhysicsClientExample::enqueueCommand(int commandId)
{
    m_userCommandRequests.push_back(commandId);
}

void PhysicsClientExample::prepareAndSubmitCommand(int commandId)
{
    
    switch (commandId)
    {
        case  CMD_LOAD_URDF:
        {
            
            b3SharedMemoryCommandHandle commandHandle = b3LoadUrdfCommandInit(m_physicsClientHandle, "kuka_iiwa/model.urdf");
            
            //setting the initial position, orientation and other arguments are optional
            double startPosX = 0;
            static double startPosY = 0;
            double startPosZ = 0;
            b3LoadUrdfCommandSetStartPosition(commandHandle, startPosX,startPosY,startPosZ);
			startPosY += 2.f;
//            ret = b3LoadUrdfCommandSetUseFixedBase(commandHandle, 1);
            b3SubmitClientCommand(m_physicsClientHandle, commandHandle);

            break;
        }
        case CMD_REQUEST_CAMERA_IMAGE_DATA:
        {
            ///request an image from a simulated camera, using a software renderer.
            
            b3SharedMemoryCommandHandle commandHandle = b3InitRequestCameraImage(m_physicsClientHandle);
            
			float viewMatrix[16];
			float projectionMatrix[16];
			this->m_guiHelper->getRenderInterface()->getActiveCamera()->getCameraProjectionMatrix(projectionMatrix);
			this->m_guiHelper->getRenderInterface()->getActiveCamera()->getCameraViewMatrix(viewMatrix);
            b3RequestCameraImageSetCameraMatrices(commandHandle, viewMatrix,projectionMatrix);
			b3RequestCameraImageSetPixelResolution(commandHandle, camVisualizerWidth,camVisualizerHeight);
			b3SubmitClientCommand(m_physicsClientHandle, commandHandle);
            break;
        }
        case CMD_CREATE_BOX_COLLISION_SHAPE:
        {
            b3SharedMemoryCommandHandle commandHandle = b3CreateBoxShapeCommandInit(m_physicsClientHandle);
            b3CreateBoxCommandSetStartPosition(commandHandle,0,0,-3);
			b3CreateBoxCommandSetColorRGBA(commandHandle,0,0,1,1);
            b3SubmitClientCommand(m_physicsClientHandle, commandHandle);
            break;
        }
		case CMD_CREATE_RIGID_BODY:
		{
			b3SharedMemoryCommandHandle commandHandle = b3CreateBoxShapeCommandInit(m_physicsClientHandle);
            b3CreateBoxCommandSetStartPosition(commandHandle,0,0,0);
			b3CreateBoxCommandSetMass(commandHandle,1);
			b3CreateBoxCommandSetCollisionShapeType(commandHandle,COLLISION_SHAPE_TYPE_CYLINDER_Y);
			b3CreateBoxCommandSetColorRGBA(commandHandle,1,1,0,1);
			double radius = 0.2;
			double halfHeight = 0.5;
			b3CreateBoxCommandSetHalfExtents(commandHandle,radius,halfHeight,radius);
            b3SubmitClientCommand(m_physicsClientHandle, commandHandle);
			break;
		}

        case CMD_REQUEST_ACTUAL_STATE:
        {
			if (m_selectedBody>=0)
			{
				b3SharedMemoryCommandHandle commandHandle = b3RequestActualStateCommandInit(m_physicsClientHandle,m_selectedBody);
                b3SharedMemoryStatusHandle statusHandle = b3SubmitClientCommandAndWaitStatus(m_physicsClientHandle, commandHandle);
				b3SubmitClientCommand(m_physicsClientHandle, commandHandle);
                
                int numJoints = b3GetNumJoints(m_physicsClientHandle, m_selectedBody);
                for (int i = 0; i < numJoints; ++i) {
                    struct b3JointSensorState sensorState;
                    b3GetJointState(m_physicsClientHandle, statusHandle, i, &sensorState);
                    b3Printf("Joint %d: %f", i, sensorState.m_jointMotorTorque);
                }
			}
            break;
        };

		case CMD_INIT_POSE:
		{
			
			if (m_selectedBody>=0)
			{
				b3SharedMemoryCommandHandle commandHandle = b3CreatePoseCommandInit(m_physicsClientHandle,m_selectedBody);
				static int toggle = 0;
				double pos[3] = {0,0,0};
				pos[toggle] = 2;
				toggle++; 
				if (toggle>2)
					toggle=0;

				btQuaternion orn;
				orn.setValue(0,0,0,1);

				switch (toggle)
				{
				case 0:
					orn = btQuaternion(btVector3(1,0,0),SIMD_HALF_PI);
					break;
				case 1:
					orn = btQuaternion(btVector3(0,1,0),SIMD_HALF_PI);
					break;
				case 2:
					orn = btQuaternion(btVector3(0,0,1),SIMD_HALF_PI);
					break;

				default:
					orn.setValue(0,0,0,1);
				};
				

				b3CreatePoseCommandSetBaseOrientation(commandHandle,orn[0],orn[1],orn[2],orn[3]);
				b3CreatePoseCommandSetBasePosition(commandHandle, pos[0],pos[1],pos[2]);
				int numJoints = b3GetNumJoints(m_physicsClientHandle,m_selectedBody);
				static double jointPos = SIMD_PI/2.f;
				
				for (int i=0;i<numJoints;i++)
				{
					b3JointInfo info;
					b3GetJointInfo(m_physicsClientHandle, m_selectedBody,i, &info);
					if ((info.m_jointType == 0) || (info.m_jointType == 1)) //revolute or prismatic
					{
						b3CreatePoseCommandSetJointPosition(m_physicsClientHandle,commandHandle,i,jointPos);
					}
				}
				jointPos += SIMD_PI/8.0;
				b3SubmitClientCommand(m_physicsClientHandle, commandHandle);
			}
			break;
		}
        case CMD_STEP_FORWARD_SIMULATION:
        {
        
            b3SharedMemoryCommandHandle commandHandle = b3InitStepSimulationCommand(m_physicsClientHandle);
            b3SubmitClientCommand(m_physicsClientHandle, commandHandle);
            break;
        }
            
        case CMD_REQUEST_DEBUG_LINES:
        {
            b3SharedMemoryCommandHandle commandHandle = b3InitRequestDebugLinesCommand(m_physicsClientHandle, btIDebugDraw::DBG_DrawWireframe);
            b3SubmitClientCommand(m_physicsClientHandle, commandHandle);
            break;
        }
        case CMD_SEND_DESIRED_STATE:
        {
            // b3SharedMemoryCommandHandle command = b3JointControlCommandInit( m_physicsClientHandle, CONTROL_MODE_VELOCITY);
            b3SharedMemoryCommandHandle command = b3JointControlCommandInit( m_physicsClientHandle, CONTROL_MODE_POSITION_VELOCITY_PD);
            prepareControlCommand(command);
            b3SubmitClientCommand(m_physicsClientHandle, command);
            break;
        }
        case CMD_RESET_SIMULATION:
        {
            b3SharedMemoryCommandHandle commandHandle = b3InitResetSimulationCommand(m_physicsClientHandle);
            b3SubmitClientCommand(m_physicsClientHandle, commandHandle);
            break;
        }
        case CMD_SEND_BULLET_DATA_STREAM:
        {
#if 0

            //this worked, but needs C-API and a streaming options, similar to debug lines
            command.m_type = buttonId;
            cl->enqueueCommand(command);
#endif

            break;
        }
        case CMD_SEND_PHYSICS_SIMULATION_PARAMETERS: {
            b3SharedMemoryCommandHandle commandHandle = b3InitPhysicsParamCommand(m_physicsClientHandle);
            b3PhysicsParamSetGravity(commandHandle, 0.0, 0.0, -9.8);
            b3SubmitClientCommand(m_physicsClientHandle, commandHandle);
            break;
        }
        default:
        {
            b3Error("Unknown buttonId");
            btAssert(0);
        }
    };
}



PhysicsClientExample::PhysicsClientExample(GUIHelperInterface* helper, int options)
:SharedMemoryCommon(helper),
m_physicsClientHandle(0),
m_wantsTermination(false),
m_sharedMemoryKey(SHARED_MEMORY_KEY),
m_selectedBody(-1),
m_prevSelectedBody(-1),
m_numMotors(0),
m_options(options),
m_isOptionalServerConnected(false),
m_canvas(0)
{
	b3Printf("Started PhysicsClientExample\n");
}

PhysicsClientExample::~PhysicsClientExample()
{
	if (m_physicsClientHandle)
	{
		b3ProcessServerStatus(m_physicsClientHandle);
		b3DisconnectSharedMemory(m_physicsClientHandle);
	}

	if (m_options == eCLIENTEXAMPLE_SERVER)
	{
		bool deInitializeSharedMemory = true;
		m_physicsServer.disconnectSharedMemory(deInitializeSharedMemory);
	}
	
	if (m_canvas && (m_canvasIndex>=0))
	{
		m_canvas->destroyCanvas(m_canvasIndex);
	}
    b3Printf("~PhysicsClientExample\n");
}

void	PhysicsClientExample::createButton(const char* name, int buttonId, bool isTrigger )
{
	ButtonParams button(name,buttonId,  isTrigger);
	button.m_callback = MyCallback;
	button.m_userPointer = this;
	m_guiHelper->getParameterInterface()->registerButtonParameter(button);
}

void	PhysicsClientExample::createButtons()
{
	bool isTrigger = false;
	
    if (m_guiHelper && m_guiHelper->getParameterInterface())
    {
		m_guiHelper->getParameterInterface()->removeAllParameters();

        createButton("Load URDF",CMD_LOAD_URDF,  isTrigger);
        createButton("Get Camera Image",CMD_REQUEST_CAMERA_IMAGE_DATA,isTrigger);
        createButton("Step Sim",CMD_STEP_FORWARD_SIMULATION,  isTrigger);
        createButton("Send Bullet Stream",CMD_SEND_BULLET_DATA_STREAM,  isTrigger);
		if (m_options!=eCLIENTEXAMPLE_SERVER)
		{
			createButton("Get State",CMD_REQUEST_ACTUAL_STATE,  isTrigger);
		}
        createButton("Send Desired State",CMD_SEND_DESIRED_STATE,  isTrigger);
        createButton("Create Box Collider",CMD_CREATE_BOX_COLLISION_SHAPE,isTrigger);
		createButton("Create Cylinder Body",CMD_CREATE_RIGID_BODY,isTrigger);
        createButton("Reset Simulation",CMD_RESET_SIMULATION,isTrigger);
		createButton("Initialize Pose",CMD_INIT_POSE,  isTrigger);
        createButton("Set gravity", CMD_SEND_PHYSICS_SIMULATION_PARAMETERS, isTrigger);


		if (m_physicsClientHandle && m_selectedBody>=0)
		{
			int numJoints = b3GetNumJoints(m_physicsClientHandle,m_selectedBody);
			for (int i=0;i<numJoints;i++)
			{
				b3JointInfo info;
				b3GetJointInfo(m_physicsClientHandle,m_selectedBody,i,&info);
				b3Printf("Joint %s at q-index %d and u-index %d\n",info.m_jointName,info.m_qIndex,info.m_uIndex);
                
				if (info.m_flags & JOINT_HAS_MOTORIZED_POWER)
				{
					if (m_numMotors<MAX_NUM_MOTORS)
					{
						char motorName[1024];
						sprintf(motorName,"%s q", info.m_jointName);
						// MyMotorInfo2* motorInfo = &m_motorTargetVelocities[m_numMotors];
                        MyMotorInfo2* motorInfo = &m_motorTargetPositions[m_numMotors];
						motorInfo->m_velTarget = 0.f;
                        motorInfo->m_posTarget = 0.f;
						motorInfo->m_uIndex = info.m_uIndex;
                        motorInfo->m_qIndex = info.m_qIndex;
                        
						// SliderParams slider(motorName,&motorInfo->m_velTarget);
						// slider.m_minVal=-4;
						// slider.m_maxVal=4;
                        SliderParams slider(motorName,&motorInfo->m_posTarget);
                        slider.m_minVal=-4;
                        slider.m_maxVal=4;
						if (m_guiHelper && m_guiHelper->getParameterInterface())
						{
							m_guiHelper->getParameterInterface()->registerSliderFloatParameter(slider);
						}
						m_numMotors++;
					}
				}
			}
		}
    }
}



void	PhysicsClientExample::initPhysics()
{
	if (m_guiHelper && m_guiHelper->getParameterInterface())
	{
		int upAxis = 2;
		m_guiHelper->setUpAxis(upAxis);

		createButtons();		
		
	} else
	{
        MyCallback(CMD_LOAD_URDF, true, this);
        MyCallback(CMD_STEP_FORWARD_SIMULATION,true,this);
        MyCallback(CMD_STEP_FORWARD_SIMULATION,true,this);
        MyCallback(CMD_RESET_SIMULATION,true,this);
	}

	m_selectedBody = -1;
	m_prevSelectedBody = -1;

	{
		m_canvas = m_guiHelper->get2dCanvasInterface();
		if (m_canvas)
		{
			

			m_canvasIndex = m_canvas->createCanvas("Synthetic Camera",camVisualizerWidth, camVisualizerHeight);

			for (int i=0;i<camVisualizerWidth;i++)
			{
				for (int j=0;j<camVisualizerHeight;j++)
				{
					unsigned char red=255;
					unsigned char green=255;
					unsigned char blue=255;
					unsigned char alpha=255;
					if (i==j)
					{
						red = 0;
						green=0;
						blue=0;
					}
					m_canvas->setPixel(m_canvasIndex,i,j,red,green,blue,alpha);
				}
			}
			m_canvas->refreshImageData(m_canvasIndex);
			
		}

	}

    if (m_options == eCLIENTEXAMPLE_SERVER)
    {
        m_isOptionalServerConnected = m_physicsServer.connectSharedMemory( m_guiHelper);
    }
    
	if (m_options == eCLIENTEXAMPLE_DIRECT)
	{
		m_physicsClientHandle = b3ConnectPhysicsDirect();
	} else
	{
	    m_physicsClientHandle  = b3ConnectSharedMemory(m_sharedMemoryKey);
		//m_physicsClientHandle  = b3ConnectPhysicsLoopback(SHARED_MEMORY_KEY);
	}
	
    if (!b3CanSubmitCommand(m_physicsClientHandle))
    {
		b3Warning("Cannot connect to physics client");
	}

}


void	PhysicsClientExample::stepSimulation(float deltaTime)
{

	if (m_options == eCLIENTEXAMPLE_SERVER)
	{
		for (int i=0;i<100;i++)
		{
			m_physicsServer.processClientCommands();
		}
	}

	if (m_prevSelectedBody != m_selectedBody)
	{
		createButtons();
		m_prevSelectedBody = m_selectedBody;
	}
    
	//while (!b3CanSubmitCommand(m_physicsClientHandle))
	{
		b3SharedMemoryStatusHandle status = b3ProcessServerStatus(m_physicsClientHandle);
		bool hasStatus = (status != 0);
		if (hasStatus)
		{

			int statusType = b3GetStatusType(status);
			if (statusType == CMD_ACTUAL_STATE_UPDATE_COMPLETED)
			{
				//b3Printf("bla\n");
			}
			if (statusType ==CMD_CAMERA_IMAGE_COMPLETED)
            {
			//	static int counter=0;
			//	char msg[1024];
			//	sprintf(msg,"Camera image %d OK\n",counter++);
				b3CameraImageData imageData;
				b3GetCameraImageData(m_physicsClientHandle,&imageData);
				if (m_canvas && m_canvasIndex >=0)
				{
					for (int i=0;i<camVisualizerWidth;i++)
					{
						for (int j=0;j<camVisualizerHeight;j++)
						{
                            int xIndex = int(float(i)*(float(imageData.m_pixelWidth)/float(camVisualizerWidth)));
                            int yIndex = int(float(j)*(float(imageData.m_pixelHeight)/float(camVisualizerHeight)));
							btClamp(yIndex,0,imageData.m_pixelHeight);
							btClamp(xIndex,0,imageData.m_pixelWidth);
							int bytesPerPixel = 4; //RGBA
							
							int pixelIndex = (xIndex+yIndex*imageData.m_pixelWidth)*bytesPerPixel;
							m_canvas->setPixel(m_canvasIndex,i,j,
                                               
									imageData.m_rgbColorData[pixelIndex],
									imageData.m_rgbColorData[pixelIndex+1],
									imageData.m_rgbColorData[pixelIndex+2],
                                               255); //alpha set to 255
						}
					}
					m_canvas->refreshImageData(m_canvasIndex);
				}

               // b3Printf(msg);
            } 
            if (statusType == CMD_CAMERA_IMAGE_FAILED)
            {
                b3Warning("Camera image FAILED\n");
            }
       
        
      		if (statusType == CMD_URDF_LOADING_COMPLETED)
			{
				int bodyIndex = b3GetStatusBodyIndex(status);
				if (bodyIndex>=0)
				{
					int numJoints = b3GetNumJoints(m_physicsClientHandle,bodyIndex);
            
					for (int i=0;i<numJoints;i++)
					{
						b3JointInfo info;
						b3GetJointInfo(m_physicsClientHandle,bodyIndex,i,&info);
						b3Printf("Joint %s at q-index %d and u-index %d\n",info.m_jointName,info.m_qIndex,info.m_uIndex);
				
					}
					ComboBoxParams comboParams;
					comboParams.m_comboboxId = bodyIndex;
					comboParams.m_numItems = 1;
					comboParams.m_startItem = 0;
					comboParams.m_callback = MyComboBoxCallback;
					comboParams.m_userPointer = this;
					const char* bla = "bla";
					const char* blarray[1];
					blarray[0] = bla;
				
					comboParams.m_items=blarray;//{&bla};
					m_guiHelper->getParameterInterface()->registerComboBox(comboParams);
		

				}

			}
    
		}
	}
    if (b3CanSubmitCommand(m_physicsClientHandle))
    {
        if (m_userCommandRequests.size())
        {
            //b3Printf("Outstanding user command requests: %d\n", m_userCommandRequests.size());
            int commandId = m_userCommandRequests[0];

            //a manual 'pop_front', we don't use 'remove' because it will re-order the commands
            for (int i=1;i<m_userCommandRequests.size();i++)
            {
                m_userCommandRequests[i-1] = m_userCommandRequests[i];
            }

            m_userCommandRequests.pop_back();
            
            //for the CMD_RESET_SIMULATION we need to do something special: clear the GUI sliders
            if (commandId ==CMD_RESET_SIMULATION)
            {
				m_selectedBody = -1;
                m_numMotors=0;
                createButtons();
				b3SharedMemoryCommandHandle commandHandle = b3InitResetSimulationCommand(m_physicsClientHandle);
				if (m_options == eCLIENTEXAMPLE_SERVER)
				{
					b3SubmitClientCommand(m_physicsClientHandle, commandHandle);
					while (!b3CanSubmitCommand(m_physicsClientHandle))
					{
						m_physicsServer.processClientCommands();
						b3SharedMemoryStatusHandle status = b3ProcessServerStatus(m_physicsClientHandle);
						bool hasStatus = (status != 0);
						if (hasStatus)
						{
							//int statusType = b3GetStatusType(status);
							//b3Printf("Status after reset: %d",statusType);
						}
					}
				} else
				{
					prepareAndSubmitCommand(commandId);
				}
            } else
			{
	            prepareAndSubmitCommand(commandId);
			}
            
        }  else
        {
            if (m_numMotors)
            {
                enqueueCommand(CMD_SEND_DESIRED_STATE);
                enqueueCommand(CMD_STEP_FORWARD_SIMULATION);
				if (m_options != eCLIENTEXAMPLE_SERVER)
				{
					enqueueCommand(CMD_REQUEST_DEBUG_LINES);
				}
                //enqueueCommand(CMD_REQUEST_ACTUAL_STATE);
            }
        }
    }


}

extern int gSharedMemoryKey;


class CommonExampleInterface*    PhysicsClientCreateFunc(struct CommonExampleOptions& options)
{
    PhysicsClientExample* example = new PhysicsClientExample(options.m_guiHelper, options.m_option);
	if (gSharedMemoryKey>=0)
	{
		example->setSharedMemoryKey(gSharedMemoryKey);
	}
	return example;
}
