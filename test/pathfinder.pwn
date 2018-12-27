/*
	- This is an experimental pathfinder made for SA-MP totally in pawn, the same is still not recommended for use.
	- Author: ForT
	- 2018
*/

#include <a_samp>
#include <zcmd>
#include <sscanf2>
#include <colandreas>
#include <streamer>
#include <a_pathfinder>

#define LIMIT_NODES 500

new
	Float:px,
	Float:py,
	Float:pz;

CMD:position(playerid) {

	GetPlayerPos(playerid, px, py, pz);
    
    new str[75];
    format(str, sizeof str, "The end position is here: %f %f %f", px, py, pz);
    SendClientMessage(playerid, -1, str);
    
	return 1;
}

CMD:pathfinder(playerid, params[])
{
	if (sscanf(params, "f", Float:params[0]))
	    return SendClientMessage(playerid, -1, "/pathfinder <stepsize>");

	new
		Float:x,
		Float:y,
		Float:z,
		tick = GetTickCount()
		;
		
	GetPlayerPos(playerid, x, y, z);
    
    new cyPath[LIMIT_NODES];
    
    new countNodes = CY_FindPath(x, y, z, px, py, pz, cyPath, .step_size = Float: params[0]);
    
    if (countNodes)
	{
        new id;
        while (cyPath[id]) {
            CY_GetNodePosition(cyPath[id], x, y, z);
        
            CreateDynamicObject(19135, x, y, z, 0.0, 0.0, 0.0);
            CreateDynamicMapIcon(x, y, z, 0, 0xFF0000FF);
            id++;
        }
    }
    new str[75];
    SendClientMessage(playerid, -1, "----------------------------------------------------------");
    format(str, sizeof str, "%s: {FFFFFF} Nodes: %d", countNodes ? ("{00AA00}SUCCESS") : ("{FF0000}ERROR"), countNodes);
    SendClientMessage(playerid, -1, str);
    format(str, sizeof str, "Start: %f %f %f", x, y, z);
    SendClientMessage(playerid, -1, str);
    format(str, sizeof str, "End: %f %f %f", px, py, pz);
    SendClientMessage(playerid, -1, str);
    format(str, sizeof str, "Time: %dms", GetTickCount() - tick);
    SendClientMessage(playerid, -1, str);
    SendClientMessage(playerid, -1, "----------------------------------------------------------");
    
	return 1;
}
