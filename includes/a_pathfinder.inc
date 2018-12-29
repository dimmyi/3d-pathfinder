/*
	- This is an experimental pathfinder made for SA-MP totally in pawn, the same is still not recommended for use.
	- Author: ForT
	- 2018
*/

#if defined _pathfinder_include
	#endinput
#endif
#define _pathfinder_include

#if !defined COLANDREAS
	#error [a_pathfinder] need the ColAndreas plugin to work, please add "#include <ColAndreas>" in your script
#endif

// definitions

#define CY_MAX_NODES 				4000 // nodes limit number
#define CY_MAX_COLLIDE_OBJECTS 		8    // max collision number of raycast multiline (get all collisions from top to bottom and turns them into nodes)
#define CY_MAX_NODES_CONNECTIONS    8    // limit number of neighbors for each node
#define CY_MAX_PATH_DISTANCE        75.0 // distance limit to calculate a path

// utils
#define gDistanceBetweenNodes(%0,%1) \
	VectorSize(gNodesPositions[%0][0]-gNodesPositions[%1][0],gNodesPositions[%0][1]-gNodesPositions[%1][1],gNodesPositions[%0][2]-gNodesPositions[%1][2])

#define gDistanceBetweenPoints(%1,%2,%3,%4,%5,%6) \
	VectorSize(%1-%4,%2-%5,%3-%6)

#define gAngleBetweenPoints(%0,%1,%2,%3) \
	(180.0-atan2(%0-%2,%1-%3))

enum CY_PATH_INFO {
	Float:cy_StartPosX,
	Float:cy_StartPosY,
	Float:cy_StartPosZ,
	Float:cy_EndPosX,
	Float:cy_EndPosY,
	Float:cy_EndPosZ,
	Float:cy_StepSize,
	cy_MaxNodes,
}

// count nodes
static CY_PATHFINDER_COUNT = 0;

// Parents and nodes
static Float:gNodesPositions[CY_MAX_NODES][3];
static gNodesConnections[CY_MAX_NODES][CY_MAX_NODES_CONNECTIONS];
static gNodesPath[CY_PATH_INFO];

static Float: gNodesPriorityG[CY_MAX_NODES];
static Float: gNodesPriorityF[CY_MAX_NODES];
static bool: gNodesClosedList[CY_MAX_NODES char];
static bool: gNodesOpenList[CY_MAX_NODES char];
static gNodesPreviewList[CY_MAX_NODES];

stock CY_FindPath(Float:startX, Float:startY, Float:startZ, Float:endX, Float:endY, Float:endZ, nodesPathPreview[], Float:step_size = 3.0, len_max = sizeof (nodesPathPreview))
{
    CY_PATHFINDER_COUNT = 1;

    new start_node_id = CY_AddNodeOnly(startX, startY, startZ);

 	new Float:distance_between_points = gDistanceBetweenPoints(startX, startY, startZ, endX, endY, endZ);
	new Float:angle_between_points = gAngleBetweenPoints(startX, startY, endX, endY);

	gNodesPath[cy_StartPosX] 	= startX + ((distance_between_points / 2.0) * floatsin(-angle_between_points, degrees));
	gNodesPath[cy_StartPosY] 	= startY + ((distance_between_points / 2.0) * floatcos(-angle_between_points, degrees));
	gNodesPath[cy_StartPosZ] 	= startZ;
	gNodesPath[cy_EndPosX] 		= endX;
	gNodesPath[cy_EndPosY] 		= endY;
	gNodesPath[cy_EndPosZ] 		= endZ;
	gNodesPath[cy_StepSize] 	= step_size;
	gNodesPath[cy_MaxNodes]    	= len_max;
	
	CY_CreateSpiral(gNodesPath[cy_StartPosX], gNodesPath[cy_StartPosY], gNodesPath[cy_StartPosZ], step_size, .range = distance_between_points * 1.5);

	new end_node_id = CY_AddNodeOnly(endX, endY, endZ);

	return CY_AStar(end_node_id, start_node_id, nodesPathPreview);
}

stock CY_CreateSpiral(Float:x, Float:y, Float:z, Float:stepsize = 3.0, Float: range = 50.0)
{
	new id = 0, i = 0;

	new Float:sx = x;
	new Float:sy = y;
	new Float:sz = z;

	while (range > 0.0) {
	    id++;
		for (i = 0; i < id; i++, sx += stepsize) CY_AddNode(sx, sy, sz, range);

		for (i = 0; i < id; i++, sy -= stepsize) CY_AddNode(sx, sy, sz, range);

		id++;
		for (i = 0; i < id; i++, sx -= stepsize) CY_AddNode(sx, sy, sz, range);

		for (i = 0; i < id; i++, sy += stepsize) CY_AddNode(sx, sy, sz, range);

		range -= stepsize;
	}
	return CY_PATHFINDER_COUNT;
}

stock CY_AddNode(Float:x, Float:y, Float:z, Float: maxRange, Float:max_zdiff = 20.0)
{
	if (VectorSize(x - gNodesPath[cy_StartPosX], y - gNodesPath[cy_StartPosY], z - gNodesPath[cy_StartPosZ]) > maxRange)
	    return 0;

	static Float:stepZ[CY_MAX_COLLIDE_OBJECTS];
	static Float:stepD[CY_MAX_COLLIDE_OBJECTS];
	static stepModels[CY_MAX_COLLIDE_OBJECTS];
	static stepPoints;

	stepPoints = CA_RayCastMultiLine(x, y, z + max_zdiff, x, y, z - max_zdiff, stepZ, stepZ, stepZ, stepD, stepModels);

	if (stepPoints < 1 || CY_PATHFINDER_COUNT >= CY_MAX_NODES)
		return 0;

	for (new collision; collision < stepPoints; collision++) {

	    gNodesClosedList{CY_PATHFINDER_COUNT} = false;
	    gNodesOpenList{CY_PATHFINDER_COUNT} = false;

	    gNodesPositions[CY_PATHFINDER_COUNT][0] = x;
	    gNodesPositions[CY_PATHFINDER_COUNT][1] = y;
	    gNodesPositions[CY_PATHFINDER_COUNT][2] = stepZ[collision] + 1.0;

		for (new parentId = 0; parentId < CY_MAX_NODES_CONNECTIONS; parentId++) {
		    gNodesConnections[CY_PATHFINDER_COUNT][parentId] = 0;
		}

	    gNodesPriorityG[CY_PATHFINDER_COUNT] = gDistanceBetweenPoints(x, y, z, gNodesPositions[1][0], gNodesPositions[1][1], gNodesPositions[1][2]);
        gNodesPriorityF[CY_PATHFINDER_COUNT] = gNodesPriorityG[CY_PATHFINDER_COUNT] + gDistanceBetweenPoints(x, y, z, gNodesPath[cy_EndPosX], gNodesPath[cy_EndPosY], gNodesPath[cy_EndPosZ]);

	   	CY_PATHFINDER_COUNT++;
	}

    return CY_PATHFINDER_COUNT;
}

stock CY_AddNodeOnly(Float:x, Float:y, Float:z)
{
	new id = CY_PATHFINDER_COUNT;
    gNodesPositions[id][0] = x;
    gNodesPositions[id][1] = y;
    gNodesPositions[id][2] = z;
	gNodesClosedList{id} = false;
 	gNodesOpenList{id} = false;
	gNodesPriorityG[id] = 0.0;
  	gNodesPriorityF[id] = 0.0;

	for (new parentId = 0; parentId < CY_MAX_NODES_CONNECTIONS; parentId++) {
	    gNodesConnections[id][parentId] = 0;
	}

    CY_PATHFINDER_COUNT++;

    return id;
}

stock CY_SearchParents(a)
{
	new parentId;

	for (new b = 1; b < CY_PATHFINDER_COUNT; b++)
	{
	    if (!gNodesClosedList{b})
		{
			if ((gDistanceBetweenNodes(a, b) > (gNodesPath[cy_StepSize] * 1.5)))
			    continue;

			if ((gNodesPositions[a][2] < gNodesPositions[b][2] - 1.8) || (gNodesPositions[a][2] > gNodesPositions[b][2] + 1.8))
			    continue;

			static Float:x;

			if (CA_RayCastLine(gNodesPositions[a][0], gNodesPositions[a][1], gNodesPositions[a][2], gNodesPositions[b][0], gNodesPositions[b][1], gNodesPositions[b][2], x, x, x))
			    continue;

			if (CA_RayCastLine(gNodesPositions[b][0], gNodesPositions[b][1], gNodesPositions[b][2], gNodesPositions[a][0], gNodesPositions[a][1], gNodesPositions[a][2], x, x, x))
			    continue;

			for (parentId = 0; parentId < CY_MAX_NODES_CONNECTIONS; parentId++) {
			    if (!gNodesConnections[a][parentId]) { gNodesConnections[a][parentId] = b; break; }
			}
			
			if (gNodesConnections[a][CY_MAX_NODES_CONNECTIONS - 1] != 0) return 1;
  		}
	}
	return 1;
}

stock CY_AStar(currentNode, endNode, storePathNodes[])
{
	gNodesPriorityG[currentNode] = gNodesPriorityF[currentNode] = 0.0;

	gNodesOpenList{currentNode} = true;

 	while (currentNode != endNode)
	{
 	    currentNode = CY_GetNodeLowerDistance();

		if (!currentNode) return 0;

 	    if (currentNode == endNode)
 	        return pathGetPreviewList(storePathNodes, currentNode);

      	gNodesOpenList{currentNode} = false;
      	gNodesClosedList{currentNode} = true;

        CY_SearchParents(currentNode);

       	for (new e; e < CY_MAX_NODES_CONNECTIONS; ++e)
	    {
     		new currentParent = gNodesConnections[currentNode][e];

			if (currentParent == 0 || gNodesClosedList{currentParent}) continue;

			new Float: newDistance = gNodesPriorityG[currentNode] + gDistanceBetweenNodes(currentNode, currentParent);

			if (!gNodesOpenList{currentParent})
   			 	gNodesOpenList{currentParent} = true;
			else if (newDistance >= gNodesPriorityG[currentParent])
			    continue;

			gNodesPreviewList[currentParent] = currentNode;
			gNodesPriorityG[currentParent] 	= newDistance;
			gNodesPriorityF[currentParent] 	= newDistance + gDistanceBetweenNodes(currentParent, endNode);
   		}
 	}

 	return 0;
}

stock pathGetPreviewList(cyPreviewNodes[], current)
{
	new currentId = 0;

	while (gNodesPreviewList[current])
	{
		if (currentId >= gNodesPath[cy_MaxNodes] - 1)
		    return currentId - 1;

	    if (!gNodesOpenList{current})
			cyPreviewNodes[currentId++] = current;

		current = gNodesPreviewList[current];
	}

	cyPreviewNodes[currentId++] = current;

	return currentId;
}

stock CY_GetNodeLowerDistance()
{
	new nodeid = 0,
		Float:dist = 99999.0;

	for (new i = 1; i < CY_PATHFINDER_COUNT; i++) {
	    // check if node are visitted or lowest distance to destination
		if (!gNodesOpenList{i}) continue;
		// store id ofnode
		if (gNodesPriorityF[i] < dist) {
			dist = gNodesPriorityF[i];
			nodeid = i;
		}
	}
	return nodeid;
}

stock CY_GetNodePosition(id, &Float:x, &Float:y, &Float:z) {
	x = gNodesPositions[id][0];
	y = gNodesPositions[id][1];
	z = gNodesPositions[id][2];
}

