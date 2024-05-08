#include <osg/Geode>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osg/ShapeDrawable>
#include <osgGA/TrackballManipulator>



#include <osg/Geode>
#include <osg/Geometry>
#include <osgViewer/Viewer>



osg::ref_ptr<osg::Geometry> createColoredCube(float size) {
    osg::ref_ptr<osg::Geometry> cubeGeometry = new osg::Geometry();
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();


    float halfSize = size * 0.5f;
    vertices->push_back(osg::Vec3(-halfSize, -halfSize, halfSize));
    vertices->push_back(osg::Vec3( halfSize, -halfSize, halfSize));
    vertices->push_back(osg::Vec3( halfSize,  halfSize, halfSize));
    vertices->push_back(osg::Vec3(-halfSize,  halfSize, halfSize));
    vertices->push_back(osg::Vec3(-halfSize, -halfSize, -halfSize));
    vertices->push_back(osg::Vec3( halfSize, -halfSize, -halfSize));
    vertices->push_back(osg::Vec3( halfSize,  halfSize, -halfSize));
    vertices->push_back(osg::Vec3(-halfSize,  halfSize, -halfSize));

    cubeGeometry->setVertexArray(vertices);


    colors->push_back(osg::Vec4(1, 0, 0, 1)); // Rojo
    colors->push_back(osg::Vec4(0, 1, 0, 1)); // Verde
    colors->push_back(osg::Vec4(0, 0, 1, 1)); // Azul
    colors->push_back(osg::Vec4(1, 1, 0, 1)); // Amarillo
    colors->push_back(osg::Vec4(1, 0, 1, 1)); // Magenta
    colors->push_back(osg::Vec4(0, 1, 1, 1)); // Cian
    colors->push_back(osg::Vec4(0.5, 0.5, 0.5, 1)); // Gris
    colors->push_back(osg::Vec4(1, 0.5, 0, 1)); // Naranja

    cubeGeometry->setColorArray(colors, osg::Array::BIND_PER_VERTEX);

    osg::ref_ptr<osg::DrawElementsUInt> indices = new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES);

    // Frente
    indices->push_back(0); indices->push_back(1); indices->push_back(2);
    indices->push_back(2); indices->push_back(3); indices->push_back(0);

    // Derecha
    indices->push_back(1); indices->push_back(5); indices->push_back(6);
    indices->push_back(6); indices->push_back(2); indices->push_back(1);

    // Atrás
    indices->push_back(5); indices->push_back(4); indices->push_back(7);
    indices->push_back(7); indices->push_back(6); indices->push_back(5);

    // Izquierda
    indices->push_back(4); indices->push_back(0); indices->push_back(3);
    indices->push_back(3); indices->push_back(7); indices->push_back(4);

    // Abajo
    indices->push_back(4); indices->push_back(5); indices->push_back(1);
    indices->push_back(1); indices->push_back(0); indices->push_back(4);

    // Arriba
    indices->push_back(3); indices->push_back(2); indices->push_back(6);
    indices->push_back(6); indices->push_back(7); indices->push_back(3);

    cubeGeometry->addPrimitiveSet(indices.get());

    return cubeGeometry;
}





class SpinCallback : public osg::NodeCallback {
public:
    virtual void operator()(osg::Node* node, osg::NodeVisitor* nv) {
        osg::MatrixTransform* transform = dynamic_cast<osg::MatrixTransform*>(node);
        if (transform) {
            double time = osg::Timer::instance()->tick();
            double angleX = sin(time * 0.0000001) * osg::PI * 0.25; // X
            double angleY = cos(time * 0.0000001) * osg::PI * 0.25; // Y
            double angleZ = sin(time * 0.0000001) * cos(time * 0.0000001) * osg::PI * 0.25; // Z

            osg::Matrix rotationX = osg::Matrix::rotate(angleX, osg::Vec3(1.0f, 0.0f, 0.0f));
            osg::Matrix rotationY = osg::Matrix::rotate(angleY, osg::Vec3(0.0f, 1.0f, 0.0f));
            osg::Matrix rotationZ = osg::Matrix::rotate(angleZ, osg::Vec3(0.0f, 0.0f, 1.0f));

            transform->setMatrix(rotationX * rotationY * rotationZ);
        }
        traverse(node, nv);
    }
};



int main() {
    float cubeSize = 1.0f; 


    osg::ref_ptr<osg::Geometry> coloredCube = createColoredCube(cubeSize);
    osg::ref_ptr<osg::Geode> cubeGeode = new osg::Geode();
    cubeGeode->addDrawable(coloredCube.get());

    osg::ref_ptr<osg::MatrixTransform> cubeTransform = new osg::MatrixTransform();
    cubeTransform->addChild(cubeGeode.get());
    cubeTransform->setUpdateCallback(new SpinCallback());

    osg::ref_ptr<osg::Group> root = new osg::Group();
    root->addChild(cubeTransform.get());

    //Iluminación básica
    root->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON);

    osgViewer::Viewer viewer;
    viewer.setSceneData(root.get());
    viewer.setCameraManipulator(new osgGA::TrackballManipulator());
    viewer.realize();

    while (!viewer.done()) {
        viewer.frame();
    }

    return 0;
}
