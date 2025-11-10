import networkx as nx
import numpy as np
from gcs_solver import GcsEdgeInfo, GcsVertexInfo, GraphOfConvexSets
from pydrake.all import MathematicalProgram

from .drake_prog_to_gcs_solver_conic import drake_prog_to_gcs_solver_conic


def drake_prog_nx_graph_to_gcs_solver_gcs(nx_graph: nx.DiGraph) -> GraphOfConvexSets:
    """
    Converts a nx_graph where every node and edge has an attribute 'prog' of type
    MathematicalProgram to a GraphOfConvexSets. Add the attribute 'drake_standard_form'
    and 'gcs_conic' to each node and edge as attributes. Also add `gcs_object` to each node and edge.
    """
    gcs = GraphOfConvexSets()
    gcs_vertex_to_nx_vertex = dict()

    for node, data in nx_graph.nodes(data=True):
        prog: MathematicalProgram = data["prog"]
        name: str = data.get("name", str(node))
        gcs_prog, drake_standard = drake_prog_to_gcs_solver_conic(prog)
        data["drake_standard_form"] = drake_standard
        data["gcs_conic"] = gcs_prog
        data["gcs_object"] = gcs.AddVertex(gcs_prog, name=name)

        gcs_vertex_to_nx_vertex[data["gcs_object"].id()] = node

    for u, v, data in nx_graph.edges(data=True):
        u_gcs = nx_graph.nodes[u]["gcs_object"]
        v_gcs = nx_graph.nodes[v]["gcs_object"]

        drake_prog: MathematicalProgram = data["prog"].Clone()
        drake_u_conic_vars = nx_graph.nodes[u]["drake_standard_form"].x()
        drake_v_conic_vars = nx_graph.nodes[v]["drake_standard_form"].x()
        drake_uv_vars = [var for var in drake_u_conic_vars] + [
            var for var in drake_v_conic_vars
        ]
        drake_prog.SortDecisionVariables(drake_uv_vars)
        gcs_prog, drake_standard = drake_prog_to_gcs_solver_conic(drake_prog)
        data["drake_standard_form"] = drake_standard
        data["gcs_conic"] = gcs_prog
        u_name = nx_graph.nodes[u].get("name", str(u))
        v_name = nx_graph.nodes[v].get("name", str(v))
        data["gcs_object"] = gcs.AddEdge(
            gcs_prog, u_gcs, v_gcs, name=data.get("name", f"{u_name}->{v_name}")
        )

    return gcs, gcs_vertex_to_nx_vertex
