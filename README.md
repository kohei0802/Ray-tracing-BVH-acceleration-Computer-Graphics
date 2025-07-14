# Ray-tracing - Computer Graphics
My solution to Games101 HW6

# My main tasks
1. Implement BVH (bounding volume hierarchy), a kind of spatial partition methods that allow more efficient detection between ray and object. 
2. Implement logic to check whether the ray is intersecting with the AABB (axis aligned bounding box).
3. Fix bad coding practices in the original code.

# Result

<img width="987" height="678" alt="Screenshot from 2025-07-14 23-45-14" src="https://github.com/user-attachments/assets/5fcf9fab-be6e-4a2c-ac38-354d3f294056" />

# Explanation
Since checking intersections with all the triangles for each ray is inefficient. Scientists proposed different spatial partition methods to only check triangles that are potentially
in intersection with the rays. For example, Octree, KD Trees are two different ways to create the spatial partition, so that for each ray, we only need to check the objects inside 
the bounding box that the ray intersected. This is beneficial also because AABB (axis-aligned box) is very easy to test the intersection against. The method used in this code was BVH, 
which splits the objects into sub bounding boxes. This is better than the other methods in many cases because only the leaves stores the triangle, each internal node only has at most 
2 children nodes, and the fact that every triangle is in exactly one leaf node in the BVH tree. In contrast, if you use KD tree to split the space, the objects can usually end up in
more than 1 bounding box, which increases the checks. 

# Prerequisites
You need OpenCV to build this project. 
If you're on Ubuntu, run 
1. sudo apt install libopencv-dev

# Build
then, go into ./build/ directory or create one if it's not there and run
1. cmake ..
2. make

# Run (How to use the program?)
1. ./Raytracing
2. You'll see an image "binary.ppm" to see the result of the ray tracing.

Codes for this HW are mostly provided by Games 101 teaching team. 
