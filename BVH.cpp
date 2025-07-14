#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    // pass the objects
    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
    {
        bounds = getUnion(bounds, objects[i]->getBounds());
    }
        
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = getUnion(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
        {
            centroidBounds = getUnion(centroidBounds, objects[i]->getBounds().centroid());
        }

        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().centroid().x <
                       f2->getBounds().centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().centroid().y <
                       f2->getBounds().centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().centroid().z <
                       f2->getBounds().centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = getUnion(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}


Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const {
    Intersection closestIntersection; // Default: happened = false

    if (!node) return closestIntersection;

    // Step 1: Check if ray intersects this node's bounding box
    Vector3f invDir(1 / ray.direction.x, 1 / ray.direction.y, 1 / ray.direction.z);
    std::array<int, 3> dirIsNeg = {
        int(ray.direction.x < 0),
        int(ray.direction.y < 0),
        int(ray.direction.z < 0)
    };

    if (!node->bounds.intersectP(ray, invDir, dirIsNeg)) {
        return closestIntersection; // No intersection with this node
    }

    // Step 2: If leaf node, check its single object
    if (!node->left && !node->right) {
        if (node->object) { // Safety check
            closestIntersection = node->object->getIntersection(ray);
        }
        return closestIntersection;
    }

    // Step 3: If internal node, recurse into left and right children
    Intersection leftIntersection = getIntersection(node->left, ray);
    Intersection rightIntersection = getIntersection(node->right, ray);

    // Return the closer intersection
    if (!leftIntersection.happened) return rightIntersection;
    if (!rightIntersection.happened) return leftIntersection;
    return (leftIntersection.distance < rightIntersection.distance) ? leftIntersection : rightIntersection;
}