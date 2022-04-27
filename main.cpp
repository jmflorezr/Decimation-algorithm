#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkNamedColors.h>
#include <vtkProperty.h>
#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <vtkTexture.h>
#include <vtkTexturedSphereSource.h>
#include <vtkCamera.h>
#include <vtkOBJReader.h>
// INCLUDES MISSING TODO
#include "iostream"
#include "string"
#define _USE_MATH_DEFINES
#include "math.h"
#include <vtkSphereSource.h>
#include <vtkTextureMapToSphere.h>
#include <vtkJPEGReader.h>
#include <vtkTransformTextureCoords.h>
#include <vtkRendererCollection.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkTransform.h>
#include <vtkGlyph3D.h>
#include <vtkArrowSource.h>
#include <vtkPolyDataNormals.h>
#include <vtkPlane.h>
#include <vtkCutter.h>
#include <vtkStripper.h>
#include <vtkKochanekSpline.h>
#include <vtkSplineFilter.h>
#include <vtkTubeFilter.h>
#include <vtkCardinalSpline.h>
#include <vtkParametricSpline.h>
#include <vtkParametricFunctionSource.h>
#include <vtkOpenGLPolyDataMapper.h>

#define vtkSPtr vtkSmartPointer
#define vtkSPtrNew(Var, Type) vtkSPtr<Type> Var = vtkSPtr<Type>::New();

using namespace std;

vtkSPtrNew(gRenWin, vtkRenderWindow);
vtkSPtrNew(planeActor, vtkActor);
vtkSPtrNew(functionSource, vtkParametricFunctionSource);
vtkSPtrNew(gRenWinInt, vtkRenderWindowInteractor);
vtkSPtrNew(renderWindow, vtkRenderWindow);

int i = 0;
double wRadius = 6.371;
double elevacion_ruta = 0.1;

void keyboardCallbackFunction(vtkObject *caller, unsigned long eventId, void *clientData, void *callData) {
	vtkSPtrNew(renderer, vtkRenderer);
	renderer = gRenWin->GetRenderers()->GetFirstRenderer();

	string keyPressed = gRenWinInt->GetKeySym();
	if (keyPressed == "n" || keyPressed == "N") {
		std::cout << "Presionando " << keyPressed << "\n";
	}
	if (keyPressed == "m" || keyPressed == "M") {
		vtkPoints* linePoints = functionSource->GetOutput()->GetPoints();
		if (i < 50) {
			i = i + 1;
		}
		else {
			i = 0;
		}
		planeActor->SetPosition(linePoints->GetPoint(i));
		planeActor->SetOrientation(0, -90 + 2 * i, 90 - i);
		renderWindow->Render();
	}
}

class Posicion {
public:
	double latitud;
	double longitud;
	double latitudRadianes;
	double longitudRadianaes;
	double* PosicionCartesiana = new double[3];

	Posicion(double _latitud, double _longitud) {
		latitud = _latitud;
		longitud = _longitud;	
		latitudRadianes = (latitud * M_PI) / 180.0;
		longitudRadianaes = (longitud * M_PI) / 180.0;
		PosicionCartesiana[0] = (wRadius + elevacion_ruta) * cos(latitudRadianes) * cos(longitudRadianaes);
		PosicionCartesiana[1] = (wRadius + elevacion_ruta) * cos(latitudRadianes) * sin(longitudRadianaes);
		PosicionCartesiana[2] = (wRadius + elevacion_ruta) * sin(latitudRadianes);
	}
};

int main() {
	vtkNew<vtkNamedColors> colors;

	//Bogotá
	Posicion bogota(4.624335, -74.063644);
	Posicion san_salvador(13.68935, -89.18718);
	Posicion chicago(41.8379, -87.6828);
	Posicion ponta_delgada(37.7494, -25.6649);
	Posicion madrid(40.416775, -3.703790);
	Posicion paris(48.864716, 2.349014);
	Posicion roma(41.902782, 12.496366);

	double* initialPos = bogota.PosicionCartesiana;	
	
	double translate[3];
	translate[0] = 7.5;
	translate[1] = 0;
	translate[2] = 0;

	vtkNew<vtkTexturedSphereSource> world;
	world->SetThetaResolution(100);
	world->SetPhiResolution(100);
	
	world->SetRadius(wRadius);

	vtkSPtrNew(imgReader, vtkJPEGReader);
	imgReader->SetFileName("Img/worldTopo.jpg");
	imgReader->Update();

	vtkNew<vtkPolyDataNormals> world_norms;
	world_norms->SetInputConnection(imgReader->GetOutputPort());


	vtkNew<vtkTexture> texture;
	texture->SetInputConnection(imgReader->GetOutputPort());	
	texture->InterpolateOn();

	vtkNew<vtkTransformTextureCoords> map_world_texture;
	map_world_texture->SetInputConnection(world->GetOutputPort());	
	map_world_texture->SetPosition(translate);

	vtkNew<vtkOpenGLPolyDataMapper> worldMapper;
	worldMapper->SetInputConnection(map_world_texture->GetOutputPort());

	vtkNew<vtkActor> worldActor;
	worldActor->SetMapper(worldMapper);
	worldActor->SetTexture(texture);

//Inicio SPLINE
	vtkNew<vtkPoints> points;

	points->InsertNextPoint(bogota.PosicionCartesiana);
	points->InsertNextPoint(san_salvador.PosicionCartesiana);
	points->InsertNextPoint(chicago.PosicionCartesiana);
	points->InsertNextPoint(ponta_delgada.PosicionCartesiana);
	points->InsertNextPoint(madrid.PosicionCartesiana);
	points->InsertNextPoint(paris.PosicionCartesiana);
	points->InsertNextPoint(roma.PosicionCartesiana);

	vtkNew<vtkParametricSpline> spline;
	spline->SetPoints(points);

	functionSource->SetParametricFunction(spline);
	functionSource->Update();

	vtkNew<vtkParametricFunctionSource> functionSource;
	functionSource->SetParametricFunction(spline);
	functionSource->Update();

	vtkNew<vtkPolyDataMapper> pathMapper;
	pathMapper->SetInputConnection(functionSource->GetOutputPort());

	vtkNew<vtkActor> pathActor;
	pathActor->SetMapper(pathMapper);
	pathActor->GetProperty()->SetColor(1, 0, 0);
//Fin SPLINE

//Inicio representación del avión
	vtkSPtrNew(paneReader, vtkOBJReader);
	paneReader->SetFileName ("Img/plane.obj");

	vtkNew<vtkPolyDataMapper> planeMapper;
	planeMapper->SetInputConnection(paneReader->GetOutputPort());
	
	planeActor->SetMapper(planeMapper);
	planeActor->SetScale(0.00007);
	planeActor->GetProperty()->SetColor(1, 1, 1);
	planeActor->GetProperty()->SetAmbient(0.5);
	planeActor->GetProperty()->SetDiffuse(0.7);
	planeActor->SetPosition(initialPos);
	planeActor->SetOrientation(-7.188945, -102.518610, -87.013187);
//Fin representación del avión

	vtkSPtrNew(renderer, vtkRenderer);
	renderer->AddActor(pathActor);
	renderer->AddActor(worldActor);	
	renderer->AddActor(planeActor);
	renderer->SetBackground (colors->GetColor3d("Black").GetData());

	vtkSPtrNew(keyboardCallback, vtkCallbackCommand);
	keyboardCallback->SetCallback (keyboardCallbackFunction);
	
	renderWindow->AddRenderer (renderer);
	renderWindow->SetSize (1200,800);
	gRenWin = renderWindow;

	vtkSPtrNew(interactor, vtkRenderWindowInteractor);
	interactor->SetRenderWindow (renderWindow);
	interactor->AddObserver (vtkCommand::KeyPressEvent, keyboardCallback);
	gRenWinInt = interactor;

	vtkSPtrNew(camera, vtkCamera);
	camera = renderer->GetActiveCamera();
	camera->SetPosition(initialPos[0] * 5, initialPos[1] * 5, initialPos[2] * 5);
	camera->SetFocalPoint(0, 0, 0);
	camera->Roll(-90);
	renderWindow->Render();
	interactor->Start();

	return 0;
}