#include "stdafx.h"
#include "ElementMovement.h"
Simulator::Simulator(){
	//benchmark1();
	example();
}


AgentsSequenceMovements   Simulator::intraLoopElementsExchange(size_t cID0, size_t cID1){

	AgentsSequenceMovements  sequenceMovements;
	Vector2 p0 = elements_[cID0].position;
	Vector2 p1 = elements_[cID1].position;

	vector<agentMovement> m0;
	agentMovement m00,m01;
	
	/*exchange the elements' occupied agentsId */
	AgentId aID0 = elements_[cID0].occupiedAgentID;
	AgentId aID1 = elements_[cID1].occupiedAgentID;
	if (-1 != aID0){
		elements_[cID1].occupiedAgentID = aID0;
		agent_[aID0] = cID1;

		m00.ID = cID0;
		m00.subGoal = elements_[cID1].position;
		m0.push_back(m00);
	}
	if (-1 != aID1){
		elements_[cID0].occupiedAgentID = aID1;
		agent_[aID1] = cID0;
		m01.ID = cID1;
		m01.subGoal = elements_[cID0].position;
		m0.push_back(m01);
	}
	sequenceMovements.agentMovement.push_back(m0);
	return sequenceMovements;
}
AgentsSequenceMovements  Simulator::interLoopElementsExchange(size_t cID0, size_t cID1){
	AgentsSequenceMovements  sequenceMovements;
	Vector2 p0 = elements_[cID0].position;
	Vector2 p1 = elements_[cID1].position;

	vector<agentMovement> m0;
	agentMovement m00,m01;

	/*exchange the elements' occupied agentsId */
	AgentId aID0 = elements_[cID0].occupiedAgentID;
	AgentId aID1 = elements_[cID1].occupiedAgentID;
	if (-1 != aID0){
		elements_[cID1].occupiedAgentID = aID0;
		agent_[aID0] = cID1;

		m00.ID = cID0;
		m00.subGoal = elements_[cID1].position;
		m0.push_back(m00);
	}
	if (-1 != aID1){
		elements_[cID0].occupiedAgentID = aID1;
		agent_[aID1] = cID0;
		m01.ID = cID1;
		m01.subGoal = elements_[cID0].position;
		m0.push_back(m01);
	}
	sequenceMovements.agentMovement.push_back(m0);
	return sequenceMovements;
}
/************************************************************************/
/* EXAMPLE                 
bool occupied;

size_t loopID;
size_t clusterID;
float r;//radius
Vector2 position;
vector<size_t> connectionsID;// the elements ID connect with this element.                                                     */
/************************************************************************/

void Simulator::example(){
	/*elements*/
	const size_t nLoops = 3;
	Element e;
	e.loopID = 0;
	e.clusterID = 0;
	e.occupiedAgentID = agent_.size();
	agent_.push_back(elements_.size());
	e.position = Vector2(0,0);
	e.r = agentRadius;
	elements_.push_back(e);
	

	for (size_t l = 0;l < nLoops - 1;l++ ){
		float circleRadius = 2* agentRadius * (l+1);
		float sliceAngle = 2*PI / ((l+1) * NCONNECT);
		for ( size_t i = 0;i < NCONNECT*(l+1); i++){
			e.loopID = l+1;
			e.clusterID = 0;
			
			if (elements_.size() == 4){
				e.occupiedAgentID = -1;
			} 
			else{
				e.occupiedAgentID = agent_.size();
				agent_.push_back(elements_.size());
			}
			
			e.position = Vector2(2*(l+1)*agentRadius*cos(sliceAngle*i),2*(l+1)*agentRadius*sin(sliceAngle*i));
			e.r = agentRadius;
			elements_.push_back(e);
		}
	}

	/*set the connections */

	for ( size_t i = 0;i < elements_.size(); i++){
		for (size_t j = 0;j < elements_.size();j++){
			float dst = abs(elements_[i].position - elements_[j].position);
			if (j != i && 
				1.9* agentRadius <= dst &&
				2.5* agentRadius > dst ){
				elements_[i].connectionsElementsID.push_back(j);
			}
		}
	
	}

	for ( size_t i = 0;i < elements_.size(); i++){
		for (size_t j = 0;j < elements_[i].connectionsElementsID.size();j++){
			cout<<elements_[i].connectionsElementsID[j]<<" "; 
		}
		cout<<endl;
	}
	/*set up cluster*/
	Cluster c;
	Loop l0;
	Link c0;
	c0.data = 0;
	c0.next = 0;
	l0.elementsID.push_back(c0);
	c.loops.push_back(l0);
	c.v = elements_[0].position;
	c.ran = 2 * agentRadius * (nLoops * 2 - 1);

	size_t startElementID = 1;
	for (size_t lID = 1;lID < nLoops; lID++){
		Loop l;
		for (size_t eID = startElementID; eID < lID*NCONNECT + startElementID - 1; eID++){
			Link lin;
			lin.data = eID;lin.next = eID+1;
			l.elementsID.push_back(lin);
		}
		Link lin;
		lin.data = lID*NCONNECT + startElementID - 1;
		lin.next = startElementID;
		l.elementsID.push_back(lin);

		startElementID += l.elementsID.size();
		c.loops.push_back(l);
	}
	clusters_.push_back(c);
}
void Simulator::benchmark1(){
	ifstream ifile("./benchmarks/cluster.txt",ios::in);
	size_t nCluster = -1;
	ifile>>nCluster;

	for (size_t c = 0;c < nCluster;c++){
		Cluster cluster;
		size_t nLoop = -1;
		ifile>>cluster.v.x_>>cluster.v.y_>>cluster.ran>>nLoop;

		for (size_t l = 0;l < nLoop;l++){
			Loop loop;

			size_t nElements = -1;
			ifile>>nElements;

			size_t curElementsID = elements_.size();
			Link link;
			for (size_t e = curElementsID;e < nElements + curElementsID;e++){
				Element element;
				bool occupied;
				ifile>>occupied>>element.position.x_>>element.position.y_>>element.r;
				if (true == occupied){
					element.occupiedAgentID = agent_.size();
					agent_.push_back(elements_.size()-1);
				}
				else{
					element.occupiedAgentID = -1;
				}
				elements_.push_back(element);


				link.data = e;
				if (e != nElements + curElementsID - 1 ){
					link.next = e+1;
				} 
				else{
					link.next = curElementsID;
				}
				loop.elementsID.push_back(link);
			}
			cluster.loops.push_back(loop);
		}
		clusters_.push_back(cluster);
	}

}
void Simulator::intialElements(vector< vector<Element> > elements)
{

}

void Simulator::changeTheConnectionsAfterTwoElementsExchangeTheirPositions(size_t cID0, size_t cID1){
	for (size_t i = 0;i < elements_[cID0].connectionsElementsID.size();i++){
		size_t ccID0 = elements_[cID0].connectionsElementsID[i];
		if (ccID0 != cID1){
			vector<size_t>::iterator itID0 = find(elements_[ccID0].connectionsElementsID.begin(),
				elements_[ccID0].connectionsElementsID.end(), cID0);
			*itID0 = cID1;
		}

	}
	for (size_t i = 0;i < elements_[cID1].connectionsElementsID.size();i++){
		size_t ccID1 = elements_[cID1].connectionsElementsID[i];
		if (ccID1 != cID0){
			vector<size_t>::iterator itID1 = find(elements_[ccID1].connectionsElementsID.begin(),
				elements_[ccID1].connectionsElementsID.end(), cID1);
			*itID1 = cID0;
		}
	}
	vector<size_t> tConnections = elements_[cID1].connectionsElementsID;
	/*upload cID0 connections*/
	elements_[cID1].connectionsElementsID.clear();
	elements_[cID1].connectionsElementsID.push_back(cID0);

	for (size_t i = 0;i < elements_[cID0].connectionsElementsID.size();i++){
		if (cID1 != elements_[cID0].connectionsElementsID[i]){
			elements_[cID1].connectionsElementsID.push_back(elements_[cID0].connectionsElementsID[i]);
		}
	}
	/*upload cID1 connections*/
	elements_[cID0].connectionsElementsID.clear();
	elements_[cID0].connectionsElementsID.push_back(cID1);

	for (size_t i = 0;i < tConnections.size();i++){
		if (cID0 != tConnections[i]){
			elements_[cID0].connectionsElementsID.push_back(tConnections[i]);
		}
	}
}

