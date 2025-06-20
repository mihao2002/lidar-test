import SwiftUI
import ARKit
import SceneKit

struct ARViewContainer: UIViewRepresentable {
    func makeUIView(context: Context) -> ARSCNView {
        let sceneView = ARSCNView()
        sceneView.delegate = context.coordinator
        sceneView.automaticallyUpdatesLighting = true

        let config = ARWorldTrackingConfiguration()
        config.sceneReconstruction = .mesh
        config.environmentTexturing = .automatic
        config.planeDetection = [.horizontal, .vertical] // or just [.horizontal]

        // Add cumulative mesh node to the scene root
        let cumulativeMeshNode = SCNNode()
        sceneView.scene.rootNode.addChildNode(cumulativeMeshNode)
        context.coordinator.cumulativeMeshNode = cumulativeMeshNode

        sceneView.session.run(config)
        return sceneView
    }

    func updateUIView(_ uiView: ARSCNView, context: Context) {}

    func makeCoordinator() -> Coordinator {
        Coordinator()
    }

    class Coordinator: NSObject, ARSCNViewDelegate {
        var didDetect = false
        // Store the cumulative mesh node
        var cumulativeMeshNode: SCNNode?
        // Optionally, keep track of processed anchors to avoid duplicates
        var processedAnchorIdentifiers = Set<UUID>()

        func renderer(_ renderer: SCNSceneRenderer, didUpdate node: SCNNode, for anchor: ARAnchor) {
            guard let meshAnchor = anchor as? ARMeshAnchor else { return }
            guard let cumulativeMeshNode = cumulativeMeshNode else { return }
            // Optionally, avoid duplicate geometry by checking processed anchors
            // For cumulative effect, just add new geometry
            let updatedMeshNode = createWireframeMeshNode(from: meshAnchor)
            cumulativeMeshNode.addChildNode(updatedMeshNode)
        }

        func renderer(_ renderer: SCNSceneRenderer, didAdd node: SCNNode, for anchor: ARAnchor) {
            if let meshAnchor = anchor as? ARMeshAnchor {
                guard let cumulativeMeshNode = cumulativeMeshNode else { return }
                // Optionally, avoid duplicate geometry by checking processed anchors
                if processedAnchorIdentifiers.contains(meshAnchor.identifier) { return }
                processedAnchorIdentifiers.insert(meshAnchor.identifier)
                let wireframeNode = createWireframeMeshNode(from: meshAnchor)
                cumulativeMeshNode.addChildNode(wireframeNode)
            }
            
            guard let planeAnchor = anchor as? ARPlaneAnchor else { return }

            let outlineNode = createPlaneOutlineNode(from: planeAnchor)
            node.addChildNode(outlineNode)
        }
        
        func createPlaneOutlineNode(from planeAnchor: ARPlaneAnchor) -> SCNNode {
            let extent = planeAnchor.extent
            let width = extent.x
            let length = extent.z

            // Rectangle corners in local space (flat on x-z plane)
            let halfWidth = width / 2
            let halfLength = length / 2

            let corners: [SCNVector3] = [
                SCNVector3(-halfWidth, 0, -halfLength), // back left
                SCNVector3(halfWidth, 0, -halfLength),  // back right
                SCNVector3(halfWidth, 0, halfLength),   // front right
                SCNVector3(-halfWidth, 0, halfLength),  // front left
            ]

            // Create vertices
            let vertexSource = SCNGeometrySource(vertices: corners + [corners[0]]) // close the loop

            // Create line segments (5 segments to close the rectangle)
            let indices: [Int32] = [0, 1, 1, 2, 2, 3, 3, 0]
            let indexData = Data(bytes: indices, count: indices.count * MemoryLayout<Int32>.size)
            let element = SCNGeometryElement(data: indexData,
                                             primitiveType: .line,
                                             primitiveCount: indices.count / 2,
                                             bytesPerIndex: MemoryLayout<Int32>.size)

            let geometry = SCNGeometry(sources: [vertexSource], elements: [element])
            geometry.firstMaterial?.diffuse.contents = UIColor.white
            geometry.firstMaterial?.isDoubleSided = true

            let node = SCNNode(geometry: geometry)
            node.position = SCNVector3(planeAnchor.center.x, 0, planeAnchor.center.z)
            return node
        }


//        func renderer(_ renderer: SCNSceneRenderer, didAdd node: SCNNode, for anchor: ARAnchor) {
//            guard !didDetect, let meshAnchor = anchor as? ARMeshAnchor else { return }
//            didDetect = true
//
//            DispatchQueue.main.async {
//                if let sceneView = renderer as? ARSCNView {
////                    let box = SCNBox(width: 0.2, height: 0.2, length: 0.2, chamferRadius: 0)
////                    box.firstMaterial?.diffuse.contents = UIColor.green.withAlphaComponent(0.5)
////                    let boxNode = SCNNode(geometry: box)
////                    boxNode.position = SCNVector3Zero // At the anchor's origin
////                    node.addChildNode(boxNode)
//                    //                    if let bounds = self.computeBoundingBox(from: meshAnchor) {
////                        self.draw2DBoundingBox(min: bounds.min, max: bounds.max, in: sceneView)
////                        sceneView.session.pause()
////                    }
////                    if let bounds = self.computeBoundingBox(from: meshAnchor) {
////                        self.addBoundingBoxNode(min: bounds.min, max: bounds.max, to: node)
////                        sceneView.session.pause()
////                    }
//                    
//                    // Add a visible red semi-transparent cube at the anchor's origin (local coordinates)
//
//                    
////                    if let bounds = self.computeBoundingBoxLocal(from: meshAnchor) {
////                        self.addBoundingBoxNodeLocal(min: bounds.min, max: bounds.max, to: node)
////                        (renderer as? ARSCNView)?.session.pause()
////                    }
//                    let meshNode = self.createMeshNode(from: meshAnchor)
//                    node.addChildNode(meshNode)
//                    (renderer as? ARSCNView)?.session.pause()
//                }
//            }
//        }
        
        private func createWireframeMeshNode(from meshAnchor: ARMeshAnchor) -> SCNNode {
            let geometry = meshAnchor.geometry
            let vertexBuffer = geometry.vertices
            let indexBuffer = geometry.faces

            let vertexCount = vertexBuffer.count / vertexBuffer.stride
            var vertexArray: [SCNVector3] = []

            for i in 0..<vertexCount {
                let offset = vertexBuffer.offset + i * vertexBuffer.stride
                let pointer = vertexBuffer.buffer.contents().advanced(by: offset)
                let floatPtr = pointer.bindMemory(to: Float.self, capacity: 3)
                let vertex = SCNVector3(floatPtr[0], floatPtr[1], floatPtr[2])
                vertexArray.append(vertex)
            }

            let indexCount = indexBuffer.count / indexBuffer.bytesPerIndex
            let indexPtr = indexBuffer.buffer.contents().bindMemory(to: UInt32.self, capacity: indexCount)

            var edgeIndices: [Int32] = []

            for i in 0..<(indexCount / 3) {
                let i0 = Int32(indexPtr[i * 3])
                let i1 = Int32(indexPtr[i * 3 + 1])
                let i2 = Int32(indexPtr[i * 3 + 2])

                // Only keep triangles with valid indices
                guard i0 < vertexArray.count, i1 < vertexArray.count, i2 < vertexArray.count else {
                    continue
                }

                // Add 3 edges (as pairs of vertices)
                edgeIndices.append(i0); edgeIndices.append(i1)
                edgeIndices.append(i1); edgeIndices.append(i2)
                edgeIndices.append(i2); edgeIndices.append(i0)
            }

            let vertexSource = SCNGeometrySource(vertices: vertexArray)
            let indexData = Data(bytes: edgeIndices, count: edgeIndices.count * MemoryLayout<Int32>.size)

            let element = SCNGeometryElement(data: indexData,
                                             primitiveType: .line,
                                             primitiveCount: edgeIndices.count / 2,
                                             bytesPerIndex: MemoryLayout<Int32>.size)

            let meshGeometry = SCNGeometry(sources: [vertexSource], elements: [element])
            meshGeometry.firstMaterial?.diffuse.contents = UIColor.green
            meshGeometry.firstMaterial?.isDoubleSided = true

            return SCNNode(geometry: meshGeometry)
        }

        
        private func createMeshNode(from meshAnchor: ARMeshAnchor) -> SCNNode {
            let geometry = meshAnchor.geometry
            let vertexBuffer = geometry.vertices
            let indexBuffer = geometry.faces

            let vertexCount = vertexBuffer.count / vertexBuffer.stride
            var vertexArray: [SCNVector3] = []

            // Extract vertices
            for i in 0..<vertexCount {
                let offset = vertexBuffer.offset + i * vertexBuffer.stride
                let pointer = vertexBuffer.buffer.contents().advanced(by: offset)
                let floatPtr = pointer.bindMemory(to: Float.self, capacity: 3)
                let vertex = SCNVector3(floatPtr[0], floatPtr[1], floatPtr[2])
                vertexArray.append(vertex)
            }

            // Extract triangle indices with bounds check
            let indexCount = indexBuffer.count / indexBuffer.bytesPerIndex
            let indexPtr = indexBuffer.buffer.contents().bindMemory(to: UInt32.self, capacity: indexCount)

            var indices: [Int32] = []

            for i in 0..<(indexCount / 3) {
                let i0 = Int32(indexPtr[i * 3])
                let i1 = Int32(indexPtr[i * 3 + 1])
                let i2 = Int32(indexPtr[i * 3 + 2])

                guard i0 < vertexArray.count && i1 < vertexArray.count && i2 < vertexArray.count else {
                    print("⚠️ Triangle index out of bounds: \(i0), \(i1), \(i2) (vertex count: \(vertexArray.count))")
                    continue
                }

                indices.append(i0)
                indices.append(i1)
                indices.append(i2)
            }

            let vertexSource = SCNGeometrySource(vertices: vertexArray)
            let indexData = Data(bytes: indices, count: indices.count * MemoryLayout<Int32>.size)
            let element = SCNGeometryElement(data: indexData,
                                             primitiveType: .triangles,
                                             primitiveCount: indices.count / 3,
                                             bytesPerIndex: MemoryLayout<Int32>.size)

            let meshGeometry = SCNGeometry(sources: [vertexSource], elements: [element])
            meshGeometry.firstMaterial?.diffuse.contents = UIColor.cyan.withAlphaComponent(0.5)
            meshGeometry.firstMaterial?.isDoubleSided = true

            return SCNNode(geometry: meshGeometry)
        }

        private func addBoundingBoxNodeLocal(min: SIMD3<Float>, max: SIMD3<Float>, to node: SCNNode) {
            // corners in local coordinates (no transform applied)
            let corners = [
                SCNVector3(min.x, min.y, min.z),
                SCNVector3(max.x, min.y, min.z),
                SCNVector3(max.x, max.y, min.z),
                SCNVector3(min.x, max.y, min.z),
                SCNVector3(min.x, min.y, max.z),
                SCNVector3(max.x, min.y, max.z),
                SCNVector3(max.x, max.y, max.z),
                SCNVector3(min.x, max.y, max.z)
            ]

            let edges = [
                (0,1), (1,2), (2,3), (3,0),
                (4,5), (5,6), (6,7), (7,4),
                (0,4), (1,5), (2,6), (3,7)
            ]

            let boundingBoxNode = SCNNode()

            for (start, end) in edges {
                let startPoint = corners[start]
                let endPoint = corners[end]

                let edgeNode = lineNode(from: startPoint, to: endPoint)
                boundingBoxNode.addChildNode(edgeNode)
            }

            node.addChildNode(boundingBoxNode)
        }

        private func computeBoundingBoxLocal(from meshAnchor: ARMeshAnchor) -> (min: SIMD3<Float>, max: SIMD3<Float>)? {
            let geometry = meshAnchor.geometry
            let vertexCount = geometry.vertices.count / geometry.vertices.stride

            guard vertexCount > 0 else { return nil }

            var minVec = SIMD3<Float>(Float.greatestFiniteMagnitude)
            var maxVec = SIMD3<Float>(-Float.greatestFiniteMagnitude)

            for i in 0..<vertexCount {
                let vertex = geometry.vertex(at: i)
                minVec = simd_min(minVec, vertex)
                maxVec = simd_max(maxVec, vertex)
            }

            return (minVec, maxVec)
        }

        
        private func addBoundingBoxNode(min: SIMD3<Float>, max: SIMD3<Float>, to node: SCNNode) {
            // 8 corners of the box
            let corners = [
                SCNVector3(min.x, min.y, min.z),
                SCNVector3(max.x, min.y, min.z),
                SCNVector3(max.x, max.y, min.z),
                SCNVector3(min.x, max.y, min.z),
                SCNVector3(min.x, min.y, max.z),
                SCNVector3(max.x, min.y, max.z),
                SCNVector3(max.x, max.y, max.z),
                SCNVector3(min.x, max.y, max.z)
            ]
            
            // Edges as pairs of indices in corners array
            let edges = [
                (0,1), (1,2), (2,3), (3,0), // bottom rectangle
                (4,5), (5,6), (6,7), (7,4), // top rectangle
                (0,4), (1,5), (2,6), (3,7)  // vertical edges
            ]
            
            let boundingBoxNode = SCNNode()
            
            for (start, end) in edges {
                let startPoint = corners[start]
                let endPoint = corners[end]
                
                let edgeNode = lineNode(from: startPoint, to: endPoint)
                boundingBoxNode.addChildNode(edgeNode)
            }
            
            node.addChildNode(boundingBoxNode)
        }

        // Helper to create a line segment node between two points
        private func lineNode(from start: SCNVector3, to end: SCNVector3) -> SCNNode {
            let vertices: [SCNVector3] = [start, end]
            let source = SCNGeometrySource(vertices: vertices)
            let indices: [UInt8] = [0, 1]
            let element = SCNGeometryElement(indices: indices, primitiveType: .line)
            let geometry = SCNGeometry(sources: [source], elements: [element])
            geometry.firstMaterial?.diffuse.contents = UIColor.red
            geometry.firstMaterial?.isDoubleSided = true
            return SCNNode(geometry: geometry)
        }


        private func computeBoundingBox(from meshAnchor: ARMeshAnchor) -> (min: SIMD3<Float>, max: SIMD3<Float>)? {
            let geometry = meshAnchor.geometry

            let vertices = geometry.vertices
            let vertexCount = vertices.count / MemoryLayout<SIMD3<Float>>.stride

            guard vertexCount > 0 else { return nil }

            var minVec = SIMD3<Float>(Float.greatestFiniteMagnitude)
            var maxVec = SIMD3<Float>(-Float.greatestFiniteMagnitude)

            for i in 0..<vertexCount {
                let vertex = geometry.vertex(at: i)
                let world = (meshAnchor.transform * SIMD4<Float>(vertex, 1)).xyz
                minVec = simd_min(minVec, world)
                maxVec = simd_max(maxVec, world)
            }


            return (minVec, maxVec)
        }

        private func draw2DBoundingBox(min: SIMD3<Float>, max: SIMD3<Float>, in sceneView: ARSCNView) {
            let corners3D: [SIMD3<Float>] = [
                SIMD3(min.x, min.y, min.z),
                SIMD3(max.x, min.y, min.z),
                SIMD3(max.x, max.y, min.z),
                SIMD3(min.x, max.y, min.z),
                SIMD3(min.x, min.y, max.z),
                SIMD3(max.x, min.y, max.z),
                SIMD3(max.x, max.y, max.z),
                SIMD3(min.x, max.y, max.z)
            ]

            let projected = corners3D.compactMap { corner -> CGPoint? in
                let vec = SCNVector3(corner.x, corner.y, corner.z)
                let projectedPoint = sceneView.projectPoint(vec)
                return projectedPoint.z > 1 ? nil : CGPoint(x: CGFloat(projectedPoint.x), y: CGFloat(projectedPoint.y))
            }

            guard projected.count == 8 else { return }

            let xs = projected.map { $0.x }
            let ys = projected.map { $0.y }
            let rect = CGRect(x: xs.min() ?? 0, y: ys.min() ?? 0,
                              width: (xs.max() ?? 0) - (xs.min() ?? 0),
                              height: (ys.max() ?? 0) - (ys.min() ?? 0))

            let layer = CAShapeLayer()
            layer.frame = rect
            layer.borderColor = UIColor.red.cgColor
            layer.borderWidth = 3
            layer.backgroundColor = UIColor.clear.cgColor

            sceneView.layer.addSublayer(layer)
        }
    }
}

extension ARMeshGeometry {
    func vertex(at index: Int) -> SIMD3<Float> {
        let vertexPointer = vertices.buffer.contents()
        let stride = vertices.stride
        let offset = vertices.offset + index * stride
        let rawPointer = vertexPointer.advanced(by: offset)
        let floatPointer = rawPointer.bindMemory(to: Float.self, capacity: 3)
        return SIMD3(floatPointer[0], floatPointer[1], floatPointer[2])
    }
}

import simd

extension SIMD4 where Scalar == Float {
    var xyz: SIMD3<Float> {
        return SIMD3<Float>(x, y, z)
    }
}
