#include "PlasticityData.h"
#include <stdio.h> 
#include <string.h> 
#include <stdlib.h> 

static bool collect = false;
bool PlasticityData::getCollect(){
	return collect;
}
void PlasticityData::setCollect(bool v){
	collect = v;
}
static list<PlasticityData> *s_pData=NULL;
void PlasticityData::setData(list<PlasticityData> *pData){
	s_pData = pData;
}
list<PlasticityData>* PlasticityData::getData(){
	return s_pData;
}

PlasticityData::PlasticityData(char * buf){
	value = buf;
}
PlasticityData::~PlasticityData(){
}
#define B_LEN 256
static bool m_logData = false;
static FILE *fp=NULL;
char fn[B_LEN];
static int m_mode;
bool PlasticityData::getLogData(){
	return m_logData;
}
char * PlasticityData::getLogDataFilename(){
	return fn;
}
void closeLogFile(){
	if (fp){
		fclose(fp);
		fprintf(fp, "];\n");
		fp = NULL;
	}
}

void PlasticityData::setLogData(bool v){
	m_logData = v;
	if (m_logData){
		strcpy_s(fn, "waiting data");
	}
	else{
		closeLogFile();
		strcpy_s(fn, "");
	}
}

static btScalar m_time;
void PlasticityData::setTime(btScalar time){
	m_time = time;
}

bool handleOpen(int mode){
	if (m_mode != mode){
		closeLogFile();
	}
	if (NULL == fp){
		m_mode = mode;
		sprintf_s(fn, B_LEN, "d:/wrk/plog-%d.m", m_mode);
		errno_t err = fopen_s(&fp, fn, "w");
		if (!fp || err){
			if (err){
				char buffer[B_LEN];
				strerror_s(buffer,B_LEN);
				sprintf_s(fn, B_LEN, "open d:/wrk/plog-%d.m failed: %s", m_mode, buffer);
			}
			return false;
		}
		fprintf(fp, "plog%d=[\n", m_mode);
	}
	return true;
}
int itemsInRow = 3;
void wr(btScalar * f, int size, char * comment=NULL, int rowskip=1){
	for (int i = 0; i < size; i+=rowskip){
		if (i>0 && (i/rowskip)%itemsInRow == 0){
			fprintf(fp, "%% %d - %d\n", i-(itemsInRow)*rowskip, i-rowskip);
		}
		fprintf(fp, "% 10.4e,", f[i]);
	}
	if (comment != NULL){
		fprintf(fp, "%% %s", comment);
	}
	fprintf(fp, "\n");
}
int jSize = 0x123;
bool logJacobian=false;
void PlasticityData::log(btTypedConstraint::btConstraintInfo2 * info, int mode){
	if (!m_logData){
		return;
	}
	if (!handleOpen(mode)){
		return;
	}
	wr(&m_time, 1,"time");
	wr(info->m_constraintError, jSize,"ce",info->rowskip);
	if (logJacobian){
		wr(info->m_J1angularAxis, jSize, "J1a");
		wr(info->m_J1linearAxis, jSize, "J1l");
		wr(info->m_J2angularAxis, jSize, "J2a");
		wr(info->m_J2linearAxis, jSize, "J2l");
	}
	fprintf(fp, ";\n");
	fflush(fp);
}
