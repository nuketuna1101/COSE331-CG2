#include "scene.h"
#include "binary/animation.h"
#include "binary/skeleton.h"
#include "binary/player.h"

Shader* Scene::vertexShader = nullptr;
Shader* Scene::fragmentShader = nullptr;
Program* Scene::program = nullptr;
Camera* Scene::camera = nullptr;
Object* Scene::player = nullptr;
Texture* Scene::diffuse = nullptr;
Material* Scene::material = nullptr;
Object* Scene::lineDraw = nullptr;
Texture* Scene::lineColor = nullptr;
Material* Scene::lineMaterial = nullptr;
int Scene::width=0;
int Scene::height=0;

void Scene::setup(AAssetManager* aAssetManager) {
    Asset::setManager(aAssetManager);

    Scene::vertexShader = new Shader(GL_VERTEX_SHADER, "vertex.glsl");
    Scene::fragmentShader = new Shader(GL_FRAGMENT_SHADER, "fragment.glsl");

    Scene::program = new Program(Scene::vertexShader, Scene::fragmentShader);

    Scene::camera = new Camera(Scene::program);
    Scene::camera->eye = vec3(0.0f, 0.0f, 80.0f);

    Scene::diffuse = new Texture(Scene::program, 0, "textureDiff", playerTexels, playerSize);
    Scene::material = new Material(Scene::program, diffuse);
    Scene::player = new Object(program, material, playerVertices, playerIndices);
    player->worldMat = scale(vec3(1.0f / 3.0f));

    Scene::lineColor = new Texture(Scene::program, 0, "textureDiff", {{0xFF, 0x00, 0x00}}, 1);
    Scene::lineMaterial = new Material(Scene::program, lineColor);
    Scene::lineDraw = new Object(program, lineMaterial, {{}}, {{}}, GL_LINES);
}

void Scene::screen(int width, int height) {
    Scene::camera->aspect = (float) width/height;
    Scene::width = width;
    Scene::height = height;
}

void Scene::update(float deltaTime) {
    Scene::program->use();

    Scene::camera->update();

    //////////////////////////////
    /* TODO */

    // Variables related to Time
    // accumulating deltaTime :: static variable
    static float TimeAccum = 0.0f;
    TimeAccum += deltaTime;
    int int_time = floor(TimeAccum);
    int cycle = int_time % 4;
    int next_cycle = (cycle + 1) % 4;
    float tquantum = TimeAccum - int_time;
    // Variables related to Animation
    vector<mat4> BStoOS;
    vector<mat4> Anima;
    //Variables related to Skinning
    vector<Vertex> BlendedV;
    mat4 Skins[4];
    mat4 TotalSkin;

    // [1] Animation
    // Forward Kinematics: Mat Animate * Mat Parent * Mat Local
    for (int i = 0; i < 28; i++){
        float Eangles[6];           // index: (currentX, Y, Z, nextX, Y, Z)
        quat RotQuats[6];           // index: (currentX, Y, Z, nextX, Y, Z)
        quat MergedRotQ[2];         // index: (current, next)
        vector<float> first_KF = motions[cycle];            // keyframe
        vector<float> next_KF = motions[next_cycle];
        // ith XYZ Rotation in motions : for index i, mapping with motions[3*(i+1)] ~ motions[3*i + 2]
        // For Interpolation, need Pair of Keyframe
        vec3 curRotXYZ = vec3(first_KF[3 * (i+1)], first_KF[3 * (i+1) + 1], first_KF[3 * (i+1) + 2]);
        vec3 nextRotXYZ = vec3(next_KF[3 * (i+1)], next_KF[3 * (i+1) + 1], next_KF[3 * (i+1) + 2]);

        // quaternions need half Euler angles
        // Produce half Euler angles for each axis X, Y, Z, (degree to radian)
        for(int axis = 0; axis < 3; axis++){
            Eangles[axis] = radians(curRotXYZ[axis] / 2);
            Eangles[axis + 3] = radians(nextRotXYZ[axis] / 2);
        }
        // XYZ rotation using quaternions
        // quat datatype: (w, x, y, z) order
        // Rotation about VECTOR by ANGLE: quat(cos(ANGLE/2), sin(ANGLE/2) * VECTOR)
        // rotation order YXZ
        for (int order = 0; order < 2; order++){
            RotQuats[3 * order] = quat(cos(Eangles[3 * order]), sin(Eangles[3 * order]), 0.0f, 0.0f);
            RotQuats[3 * order + 1] = quat(cos(Eangles[3 * order + 1]), 0.0f, sin(Eangles[3 * order + 1]), 0.0f);
            RotQuats[3 * order + 2] = quat(cos(Eangles[3 * order + 2]), 0.0f, 0.0f, sin(Eangles[3 * order + 2]));
            MergedRotQ[order] = RotQuats[3 * order + 2] * RotQuats[3 * order] * RotQuats[3 * order + 1];
        }
        // mix function : interpolate between Time quantum
        mat4 Local = mat4_cast(mix(MergedRotQ[0], MergedRotQ[1], tquantum));

        if (i == 0) {
            // i == 0 for Root class
            mat4 T0 = translate(mix(curRotXYZ, nextRotXYZ, tquantum));
            BStoOS.push_back(mat4(1.0f));
            Anima.push_back(T0 * Local);
        } else {
            // rest of layers
            mat4 toParent = translate(jOffsets[i]);
            BStoOS.push_back(BStoOS[jParents[i]] * toParent);
            Anima.push_back(Anima[jParents[i]] * toParent * Local);
        }
    }


    // [2] Skinning
    for (int i = 0; i < playerVertices.size(); i++) {
        Vertex targetV = playerVertices[i];
        // Vertex.bone (vec4 type) : index of skinned skeleton
        for (int j = 0; j < 4; j++) {
            if (targetV.bone[j] == -1){
                // nothing :: -1 cannot be used for index
                Skins[j] = mat4(0.0f);
            } else{
                // compute v' s
                Skins[j] = targetV.weight[j] * Anima[targetV.bone[j]] * inverse(BStoOS[targetV.bone[j]]);
            }
        }
        // weight blending
        TotalSkin = Skins[0] + Skins[1] + Skins[2] + Skins[3];
        // convert rest of Vertex elements
        targetV.pos = vec3(TotalSkin * vec4(targetV.pos, 1.0f));
        targetV.nor = vec3(TotalSkin * vec4(targetV.nor, 1.0f));
        BlendedV.push_back(targetV);
    }

    /////
    // Line Drawer & Debugger
    //glLineWidth(20);
    //Scene::lineDraw->load({{vec3(-20.0f, 0.0f, 0.0f)}, {vec3(20.0f, 0.0f, 0.0f)}}, {0, 1});
    //Scene::lineDraw->draw();

    //LOG_PRINT_DEBUG("You can also debug variables with this function: %f", M_PI);
    LOG_PRINT_DEBUG("----------------");
    LOG_PRINT_DEBUG("1 - deltaTime: %f", deltaTime);
    LOG_PRINT_DEBUG("- TimeAccum: %f", TimeAccum);
    LOG_PRINT_DEBUG("- int_time: %d", int_time);
    LOG_PRINT_DEBUG("- tquantum: %f", tquantum);
    LOG_PRINT_DEBUG("- cycle: %d", cycle);

    //////////////////////////////playerVertices
    Scene::player->load(BlendedV, playerIndices);
    Scene::player->draw();
}

void Scene::mouseDownEvents(float x, float y) {
    //////////////////////////////
    /* TODO: Optional problem
     * object rotating
     * Automatically called when mouse down
     */

    //////////////////////////////
}

void Scene::mouseMoveEvents(float x, float y) {
    //////////////////////////////
    /* TODO: Optional problem
     * object rotating
     * Automatically called when mouse move
     */

    //////////////////////////////
}

