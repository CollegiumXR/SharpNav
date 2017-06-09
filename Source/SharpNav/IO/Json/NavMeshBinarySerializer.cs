using System;
using System.Collections.Generic;
using System.IO;
using System.Runtime.Serialization;

using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

using SharpNav.Collections;
using SharpNav.Geometry;
using SharpNav.Pathfinding;
using System.Reflection;

#if MONOGAME
using Vector3 = Microsoft.Xna.Framework.Vector3;
#elif OPENTK
using Vector3 = OpenTK.Vector3;
#elif SHARPDX
using Vector3 = SharpDX.Vector3;
#endif

namespace SharpNav.IO.Json
{
    public class NavMeshBinarySerializer : NavMeshSerializer
    {
        public override void Serialize(string path, TiledNavMesh mesh)
        {
            var fileStream = new FileStream(path, FileMode.Create, FileAccess.Write);
            using (var binaryWriter = new BinaryWriter(fileStream))
            {
                var header = new byte[0x10];

                binaryWriter.Write(mesh.Origin.X);
                binaryWriter.Write(mesh.Origin.Y);
                binaryWriter.Write(mesh.Origin.Z);
                binaryWriter.Write(mesh.TileWidth);
                binaryWriter.Write(mesh.TileHeight);
                binaryWriter.Write(mesh.MaxTiles);
                binaryWriter.Write(mesh.MaxPolys);
                binaryWriter.Write(mesh.Tiles.Count);

                var tiles = new List<byte[]>();

                foreach (var tile in mesh.Tiles)
                {
                    var id = mesh.GetTileRef(tile);
                    tiles.Add(SerializeMeshTile(tile, id));
                }

                foreach (var tile in tiles)
                    binaryWriter.Write(tile);
            }
        }

        private byte[] SerializeMeshTile(NavTile tile, NavPolyId id)
        {
            var memoryStream = new MemoryStream();
            using (var binaryWriter = new BinaryWriter(memoryStream))
            {
                binaryWriter.Write(id.Id);
                binaryWriter.Write(tile.Location.X);
                binaryWriter.Write(tile.Location.Y);
                binaryWriter.Write(tile.Layer);
                binaryWriter.Write(tile.Salt);
                binaryWriter.Write(tile.Bounds.Min.X);
                binaryWriter.Write(tile.Bounds.Min.Y);
                binaryWriter.Write(tile.Bounds.Min.Z);
                binaryWriter.Write(tile.Bounds.Max.X);
                binaryWriter.Write(tile.Bounds.Max.Y);
                binaryWriter.Write(tile.Bounds.Max.Z);

                var polys = new List<NavPoly>(tile.Polys);

                binaryWriter.Write(polys.Count);

                foreach (var poly in polys)
                {
                    binaryWriter.Write((byte)poly.PolyType);
                    binaryWriter.Write(poly.Links.Count);

                    foreach (var link in poly.Links)
                    {
                        binaryWriter.Write(link.Reference.Id);
                        binaryWriter.Write(link.Edge);
                        binaryWriter.Write((byte)link.Side);
                        binaryWriter.Write(link.BMin);
                        binaryWriter.Write(link.BMax);
                    }

                    var verts = new List<int>(poly.Verts);
                    binaryWriter.Write(verts.Count);

                    foreach (var vert in verts)
                        binaryWriter.Write(vert);

                    var neighbors = new List<int>(poly.Neis);
                    binaryWriter.Write(neighbors.Count);

                    foreach (var neighbor in neighbors)
                        binaryWriter.Write(neighbor);

                    if (poly.Tag == null)
                        binaryWriter.Write((byte)2);
                    else
                        binaryWriter.Write((byte)poly.Tag);

                    binaryWriter.Write(poly.VertCount);
                    binaryWriter.Write(poly.Area.Id);
                }

                var vertices = new List<Vector3>(tile.Verts);

                binaryWriter.Write(vertices.Count);

                foreach (var vertex in vertices)
                {
                    binaryWriter.Write(vertex.X);
                    binaryWriter.Write(vertex.Y);
                    binaryWriter.Write(vertex.Z);
                }

                var detailMeshes = new List<PolyMeshDetail.MeshData>(tile.DetailMeshes);

                binaryWriter.Write(detailMeshes.Count);

                foreach (var detailMesh in detailMeshes)
                {
                    binaryWriter.Write(detailMesh.VertexIndex);
                    binaryWriter.Write(detailMesh.VertexCount);
                    binaryWriter.Write(detailMesh.TriangleIndex);
                    binaryWriter.Write(detailMesh.TriangleCount);
                }

                var detailVerts = new List<Vector3>(tile.DetailVerts);

                binaryWriter.Write(detailVerts.Count);

                foreach (var detailVert in detailVerts)
                {
                    binaryWriter.Write(detailVert.X);
                    binaryWriter.Write(detailVert.Y);
                    binaryWriter.Write(detailVert.Z);
                }

                var detailTris = new List<PolyMeshDetail.TriangleData>(tile.DetailTris);

                binaryWriter.Write(detailTris.Count);

                foreach (var detailTri in detailTris)
                {
                    binaryWriter.Write(detailTri.VertexHash0);
                    binaryWriter.Write(detailTri.VertexHash1);
                    binaryWriter.Write(detailTri.VertexHash2);
                    binaryWriter.Write(detailTri.Flags);
                }

                var offMeshConnections = new List<OffMeshConnection>(tile.OffMeshConnections);

                binaryWriter.Write(offMeshConnections.Count);

                foreach (var offMeshConnection in offMeshConnections)
                {
                    // todo: serialize offMeshConnection
                }

                binaryWriter.Write(tile.BVTree.Count);

                for (var i = 0; i < tile.BVTree.Count; i++)
                {
                    var node = tile.BVTree[i];

                    binaryWriter.Write(node.Bounds.Min.X);
                    binaryWriter.Write(node.Bounds.Min.Y);
                    binaryWriter.Write(node.Bounds.Min.Z);
                    binaryWriter.Write(node.Bounds.Max.X);
                    binaryWriter.Write(node.Bounds.Max.Y);
                    binaryWriter.Write(node.Bounds.Max.Z);
                    binaryWriter.Write(node.Index);
                }

                binaryWriter.Write(tile.BvQuantFactor);
                binaryWriter.Write(tile.BvNodeCount);
                binaryWriter.Write(tile.WalkableClimb);
            }

            return memoryStream.ToArray();
        }

        public override TiledNavMesh Deserialize(string path)
        {
            using (var binaryReader = new BinaryReader(new FileStream(path, FileMode.Open, FileAccess.Read)))
            {
                var originX = binaryReader.ReadSingle();
                var originY = binaryReader.ReadSingle();
                var originZ = binaryReader.ReadSingle();
                var origin = new Vector3(originX, originY, originZ);

                var tileWidth = binaryReader.ReadSingle();
                var tileHeight = binaryReader.ReadSingle();
                var maxTiles = binaryReader.ReadSingle();
                var maxPolys = binaryReader.ReadSingle();
                var mesh = new TiledNavMesh(origin, tileWidth, tileHeight, (int)maxTiles, (int)maxPolys);

                var count = binaryReader.ReadInt32();

                for (var i = 0; i < count; i++)
                {
                    NavPolyId tileRef;
                    var stream = binaryReader.BaseStream;
                    var tile = DeserializeMeshTile(ref stream, mesh.IdManager, out tileRef);

                    mesh.AddTileAt(tile, tileRef);
                }

                return mesh;
            }
        }


        private NavTile DeserializeMeshTile(ref Stream stream, NavPolyIdManager manager, out NavPolyId refId)
        {
            NavTile tile;

            using (var binaryReader = new BinaryReader(stream))
            {
                var id = binaryReader.ReadInt32();
                refId = new NavPolyId(id);

                var locationX = binaryReader.ReadInt32();
                var locationY = binaryReader.ReadInt32();
                var location = new Vector2i(locationX, locationY);

                var layer = binaryReader.ReadInt32();

                tile = new NavTile(location, layer, manager, refId);
                tile.Salt = binaryReader.ReadInt32();
                var tileBoundsMinX = binaryReader.ReadSingle();
                var tileBoundsMinY = binaryReader.ReadSingle();
                var tileBoundsMinZ = binaryReader.ReadSingle();
                var tileBoundsMaxX = binaryReader.ReadSingle();
                var tileBoundsMaxY = binaryReader.ReadSingle();
                var tileBoundsMaxZ = binaryReader.ReadSingle();
                tile.Bounds = new BBox3(tileBoundsMinX, tileBoundsMinY, tileBoundsMinZ, tileBoundsMaxX, tileBoundsMaxY,
                    tileBoundsMaxZ);

                var polyCount = binaryReader.ReadInt32();
                var polys = new NavPoly[polyCount];
                tile.PolyCount = polyCount;
                for (var i = 0; i < polyCount; i++)
                {
                    var poly = new NavPoly();
                    poly.PolyType = (NavPolyType)binaryReader.ReadByte();

                    var polyLinksCount = binaryReader.ReadInt32();

                    for (var j = 0; j < polyLinksCount; j++)
                    {
                        var link = new Link();

                        var linkReferenceId = binaryReader.ReadInt32();
                        link.Reference = new NavPolyId(linkReferenceId);
                        link.Edge = binaryReader.ReadInt32();
                        link.Side = (BoundarySide)binaryReader.ReadByte();
                        link.BMin = binaryReader.ReadInt32();
                        link.BMax = binaryReader.ReadInt32();

                        poly.Links.Add(link);
                    }

                    var vertsCount = binaryReader.ReadInt32();
                    poly.Verts = new int[vertsCount];

                    for (var j = 0; j < vertsCount; j++)
                        poly.Verts[j] = binaryReader.ReadInt32();

                    var neighborsCount = binaryReader.ReadInt32();
                    poly.Neis = new int[neighborsCount];

                    for (var j = 0; j < neighborsCount; j++)
                        poly.Neis[j] = binaryReader.ReadInt32();

                    var tag = binaryReader.ReadByte();

                    if (tag == 2)
                        poly.Tag = null;
                    else
                        poly.Tag = (OffMeshConnectionFlags)tag;

                    poly.VertCount = binaryReader.ReadInt32();

                    var areaId = binaryReader.ReadByte();
                    poly.Area = new Area(areaId);

                    polys[i] = poly;
                }

                tile.Polys = polys;

                var vertexCount = binaryReader.ReadInt32();
                var verts = new Vector3[vertexCount];

                for (var i = 0; i < vertexCount; i++)
                {
                    var x = binaryReader.ReadSingle();
                    var y = binaryReader.ReadSingle();
                    var z = binaryReader.ReadSingle();
                    var vertex = new Vector3(x, y, z);

                    verts[i] = vertex;
                }

                tile.Verts = verts;

                var detailMeshCount = binaryReader.ReadInt32();
                var detailMeshes = new PolyMeshDetail.MeshData[detailMeshCount];

                for (var i = 0; i < detailMeshCount; i++)
                {
                    var detailMesh = new PolyMeshDetail.MeshData();
                    detailMesh.VertexIndex = binaryReader.ReadInt32();
                    detailMesh.VertexCount = binaryReader.ReadInt32();
                    detailMesh.TriangleIndex = binaryReader.ReadInt32();
                    detailMesh.TriangleCount = binaryReader.ReadInt32();

                    detailMeshes[i] = detailMesh;
                }

                tile.DetailMeshes = detailMeshes;

                var detailVertCount = binaryReader.ReadInt32();
                var detailVerts = new Vector3[detailVertCount];

                for (var i = 0; i < detailVertCount; i++)
                {
                    var x = binaryReader.ReadSingle();
                    var y = binaryReader.ReadSingle();
                    var z = binaryReader.ReadSingle();
                    var vector3 = new Vector3(x, y, z);

                    detailVerts[i] = vector3;
                }

                tile.DetailVerts = detailVerts;

                var detailTriCount = binaryReader.ReadInt32();
                var detailTris = new PolyMeshDetail.TriangleData[detailTriCount];

                for (var i = 0; i < detailTriCount; i++)
                {
                    var hash0 = binaryReader.ReadInt32();
                    var hash1 = binaryReader.ReadInt32();
                    var hash2 = binaryReader.ReadInt32();
                    var flags = binaryReader.ReadInt32();
                    var triangleData = new PolyMeshDetail.TriangleData(hash0, hash1, hash2, flags);

                    detailTris[i] = triangleData;
                }

                tile.DetailTris = detailTris;

                var offMeshConnectionCount = binaryReader.ReadInt32();
                tile.OffMeshConnectionCount = offMeshConnectionCount;

                var bvTreeCount = binaryReader.ReadInt32();
                var nodes = new BVTree.Node[bvTreeCount];

                for (var i = 0; i < bvTreeCount; i++)
                {
                    var node = new BVTree.Node();

                    node.Bounds.Min.X = binaryReader.ReadInt32();
                    node.Bounds.Min.Y = binaryReader.ReadInt32();
                    node.Bounds.Min.Z = binaryReader.ReadInt32();
                    node.Bounds.Max.X = binaryReader.ReadInt32();
                    node.Bounds.Max.Y = binaryReader.ReadInt32();
                    node.Bounds.Max.Z = binaryReader.ReadInt32();
                    node.Index = binaryReader.ReadInt32();

                    nodes[i] = node;
                }

                tile.BVTree = new BVTree(nodes);
                tile.BvQuantFactor = binaryReader.ReadSingle();
                tile.BvNodeCount = binaryReader.ReadInt32();
                tile.WalkableClimb = binaryReader.ReadSingle();
            }
            return tile;
        }
    }
}
