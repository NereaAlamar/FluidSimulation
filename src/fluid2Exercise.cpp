
#include "scene.h"
#include "pcg_solver.h"
#include <algorithm>


namespace
{

	inline int clamp(int v, int lo, int hi) {
		return std::max(std::min(hi, v), lo);
	}

	inline float lerp(float v0, float v1, float t) {
		return v0 * (1 - t) + v1 * t;
	}

	inline float bilerp(float v00, float v01, float v10, float v11, float t, float s) {
		return lerp(lerp(v00, v10, t), lerp(v01, v11, t), s);
	}

}



// advection
void Fluid2::fluidAdvection( const float dt )
{
    // ink advection
	//cuando transformamos una cantidad, recorrer todas las celdillas de esa cantidad e interpolar el proceso
    {
		//declaramos una copia
		Array2<float> inkOld(ink);

		//vamos recorriendo todas las celdillas
		for (unsigned int i = 0; i < ink.getSize().x; i++)//el tamaño se puede obtener directamente del vector porque es el tamaño de las celdas
			for (unsigned int j = 0; j < ink.getSize().y; j++) {
				Vec2 pos0 = grid.getCellPos(Index2(i, j));//obtenemos la pos del índice
				Vec2 vel0(velocityX[Index2(i,j)] + velocityX[Index2(i+1,j)]*0.5f, velocityY[Index2(i, j)] + velocityY[Index2(i, j+1)] * 0.5f);
				Vec2 pos1 = pos0 - dt * vel0;//integrar hacia atrás: posición inicial - paso de tiempo * velocidad

				Vec2 index1 = grid.getCellIndex(pos1);//obtenemos el indice de la posición

				//cantidades para la interpolación
				unsigned int i_lerp0 = clamp((int)std::floor(index1.x), 0, ink.getSize().x);
				unsigned int i_lerp1 = clamp((int)std::floor(index1.x) + 1, 0, ink.getSize().x);
				unsigned int j_lerp0 = clamp((int)std::floor(index1.y), 0, ink.getSize().y);
				unsigned int j_lerp1 = clamp((int)std::floor(index1.y) + 1, 0, ink.getSize().y);


				//porcentajes de interpolación
				float t = index1.x - i_lerp0;
				float s = index1.y - j_lerp0;

				//usar la función para interpolar
				ink[Index2(i, j)] = bilerp(inkOld[Index2(i_lerp0, j_lerp0)], 
					inkOld[Index2(i_lerp0, j_lerp1)], 
					inkOld[Index2(i_lerp1, j_lerp0)], 
					inkOld[Index2(i_lerp1, j_lerp1)], 
					t, s);
			}
    }

    // velocity advection
    {
		
		Array2<float> uCopy(velocityX);
		Array2<float> vCopy(velocityY);
		

		for (unsigned int i = 0; i < velocityX.getSize().x; i++)//el tamaño se puede obtener directamente del vector porque es el tamaño de las celdas
			for (unsigned int j = 0; j < velocityX.getSize().y; j++) {

				//obtenemos los índices de las velocidades
				unsigned int i0 = clamp(i, 0, velocityY.getSize().x - 1);
				unsigned int i1 = clamp(i - 1, 0, velocityY.getSize().x - 1);
				unsigned int j0 = clamp(j, 0, velocityY.getSize().y - 1);
				unsigned int j1 = clamp(j + 1, 0, velocityY.getSize().y - 1);

				Vec2 pos0 = grid.getFaceXPos(Index2(i, j));//obtenemos la pos del índice
				Vec2 vel0(uCopy[Index2(i, j)], (vCopy[Index2(i0, j0)]+ vCopy[Index2(i0, j1)] + vCopy[Index2(i1, j0)] + vCopy[Index2(i1, j1)])*0.25);
				Vec2 pos1 = pos0 - dt * vel0;//integrar hacia atrás: posición inicial - paso de tiempo * velocidad

				Vec2 index1 = grid.getFaceIndex(pos1, 0);//obtenemos el indice de la posición

				//cantidades para la interpolación
				unsigned int i_lerp0 = clamp((int)std::floor(index1.x), 0, velocityX.getSize().x - 1);
				unsigned int i_lerp1 = clamp((int)std::floor(index1.x) + 1, 0, velocityX.getSize().x - 1);
				unsigned int j_lerp0 = clamp((int)std::floor(index1.y), 0, velocityX.getSize().y - 1);
				unsigned int j_lerp1 = clamp((int)std::floor(index1.y) + 1, 0, velocityX.getSize().y - 1);

				//porcentajes de interpolación
				float t = index1.x - i_lerp0;
				float s = index1.y - j_lerp0;

				//usar la función para interpolar
				velocityX[Index2(i, j)] = bilerp(uCopy[Index2(i_lerp0, j_lerp0)],
					uCopy[Index2(i_lerp0, j_lerp1)],
					uCopy[Index2(i_lerp1, j_lerp0)],
					uCopy[Index2(i_lerp1, j_lerp1)],
					t, s);
			}



		for (unsigned int i = 0; i < velocityY.getSize().x; i++)//el tamaño se puede obtener directamente del vector porque es el tamaño de las celdas
			for (unsigned int j = 0; j < velocityY.getSize().y; j++) {

				//obtenemos los índices de las velocidades
				unsigned int i0 = clamp(i, 0, velocityX.getSize().x - 1);
				unsigned int i1 = clamp(i + 1, 0, velocityX.getSize().x - 1);
				unsigned int j0 = clamp(j, 0, velocityX.getSize().y - 1);
				unsigned int j1 = clamp(j - 1, 0, velocityX.getSize().y - 1);

				Vec2 pos0 = grid.getFaceYPos(Index2(i, j));//obtenemos la pos del índice
				Vec2 vel0((uCopy[Index2(i0, j0)] + uCopy[Index2(i0, j1)] + uCopy[Index2(i1, j0)] + uCopy[Index2(i1, j1)])*0.25, vCopy[Index2(i, j)]);
				Vec2 pos1 = pos0 - dt * vel0;//integrar hacia atrás: posición inicial - paso de tiempo * velocidad

				Vec2 index1 = grid.getFaceIndex(pos1, 1);//obtenemos el indice de la posición

				//cantidades para la interpolación
				unsigned int i_lerp0 = clamp((int)std::floor(index1.x), 0, velocityY.getSize().x - 1);
				unsigned int i_lerp1 = clamp((int)std::floor(index1.x) + 1, 0, velocityY.getSize().x - 1);
				unsigned int j_lerp0 = clamp((int)std::floor(index1.y), 0, velocityY.getSize().y - 1);
				unsigned int j_lerp1 = clamp((int)std::floor(index1.y) + 1, 0, velocityY.getSize().y - 1);



				//porcentajes de interpolación
				float t = index1.x - i_lerp0;
				float s = index1.y - j_lerp0;

				//usar la función para interpolar
				velocityY[Index2(i, j)] = bilerp(vCopy[Index2(i_lerp0, j_lerp0)],
					vCopy[Index2(i_lerp0, j_lerp1)],
					vCopy[Index2(i_lerp1, j_lerp0)],
					vCopy[Index2(i_lerp1, j_lerp1)],
					t, s);
			}
			
    }
}



// emission
void Fluid2::fluidEmission()
{
	if (Scene::testcase >= Scene::SMOKE)
	{
		Bbox2 emitter(Vec2(-0.1f, -2.0f), Vec2(0.1f, -1.5f));
		// emit source velocity
		{
			for (unsigned int i = 0; i < grid.getSize().x; ++i)
			{
				for (unsigned int j = 0; j < grid.getSize().y; ++j)
				{
					if (emitter.isInside(grid.getCellPos(Index2(i, j))))
					{
						velocityY[Index2(i, j)] = 1.0f;
					}
				}
			}
		}
		// emit source ink
		{
			for (unsigned int i = 0; i < grid.getSize().x; ++i)
			{
				for (unsigned int j = 0; j < grid.getSize().y; ++j)
				{
					if (emitter.isInside(grid.getCellPos(Index2(i, j))))
					{
						ink[Index2(i, j)] = 0.5f;
					}

				}
			}
		}
	}
}


// volume forces
void Fluid2::fluidVolumeForces( const float dt ) //fuerza de gravedad, definida en clase escena
													//una vez tenga la gravedad, integrar la velocidad
{
    if( Scene::testcase >= Scene::SMOKE )
    {
        // gravity

		for (int i = 0; i <= velocityY.getSize().x; i++)
		{
			for (int j = 0; j <= velocityY.getSize().y; j++)
			{
				velocityY[Index2(i, j)] += dt * Scene::kGravity;
			}
		}

    }
}

// viscosity
void Fluid2::fluidViscosity( const float dt ) //mu * laplacian de velocidades (se aplicaa escalares), es una ecuación por cada componente dela velocidad
												//hacer dos bucles diferentes para aplicar el termino a cada una de las componentes de la velocidad
{
	
	Array2<float> velXCopy(getVelocityX());
	Array2<float> velYCopy(getVelocityY());
	double velXDifferenceX = 0.0f;
	double velXDifferenceY = 0.0f;
	double velYDifferenceX = 0.0f;
	double velYDifferenceY = 0.0f;

    if( Scene::testcase >= Scene::SMOKE )
    {
		
        // viscosity
		for (int i = 0; i < velocityX.getSize().x; i++)
		{
			for (int j = 0; j < velocityX.getSize().y; j++)
			{
				unsigned int i0 = clamp(i + 1, 0, velocityX.getSize().x - 1);
				unsigned int i1 = clamp(i - 1, 0, velocityX.getSize().x - 1);
				unsigned int j0 = clamp(j + 1, 0, velocityX.getSize().y - 1);
				unsigned int j1 = clamp(j - 1, 0, velocityX.getSize().y - 1);

				float velDiffX = velXCopy[Index2(i0, j)] - 2*velXCopy[Index2(i, j)]+ velXCopy[Index2(i1, j)];
				float velDiffY = velXCopy[Index2(i, j0)] - 2*velXCopy[Index2(i, j)] + velXCopy[Index2(i, j1)];

				velocityX[Index2(i, j)] += (Scene::kViscosity / Scene::kDensity)* dt * (velDiffX + velDiffY);
			}
		}
		for (int i = 0; i < velocityY.getSize().x; i++)
		{
			for (int j = 0; j < velocityY.getSize().y; j++)
			{
				unsigned int i0 = clamp(i + 1, 0, velocityY.getSize().x - 1);
				unsigned int i1 = clamp(i - 1, 0, velocityY.getSize().x - 1);
				unsigned int j0 = clamp(j + 1, 0, velocityY.getSize().y - 1);
				unsigned int j1 = clamp(j - 1, 0, velocityY.getSize().y - 1);

				float velDiffX = velYCopy[Index2(i0, j)] -2 * velYCopy[Index2(i, j)] + velYCopy[Index2(i1, j)];
				float velDiffY = velYCopy[Index2(i, j0)] - 2 * velYCopy[Index2(i, j)] + velYCopy[Index2(i, j1)];

				velocityY[Index2(i, j)] += (Scene::kViscosity / Scene::kDensity)* dt * (velDiffX + velDiffY);
			}
		}

    }
	
}

// pressure socorro perro atacando teclado S O S
void Fluid2::fluidPressureProjection(const float dt) //aplicar ambos tipos de condiciones de contorno
														//de tipo dirichlet y neuman
														//tratarla como que los bordes sean solidos (neuman) y arriba condiciones de tipo dirichlet (abierto)
{

	if (Scene::testcase >= Scene::SMOKE)
	{
		// pressure

		//Iniciamos las variables
		SparseMatrix<double> A(grid.getSize().x * grid.getSize().y, 5);
		std::vector<double> p;
		p.resize(pressure.getSize().x * pressure.getSize().y);
		std::vector<double> b(pressure.getSize().x * pressure.getSize().y);
		PCGSolver<double> solver;

		//Condiciones de contorno en las velocidades: lados y suelo
		for (int i = 0; i < velocityX.getSize().y; i++)
		{
			velocityX[Index2(0, i)] = 0.0f;
			velocityX[Index2(velocityX.getSize().x - 1, i)] = 0.0f;
		}
		for (int j = 0; j < velocityY.getSize().x; j++)
		{
			velocityY[Index2(j, 0)] = 0.0f;
		}

		//Rellenar b segun los valores de velocidad de cada una de las celdas
		for (int i = 0; i < pressure.getSize().x; i++)
		{
			for (int j = 0; j < pressure.getSize().y; j++)
			{

				//Usamos getLinearIndex para obtener el indice correcto unidimiensional según las coordenadas bidimensionales
				double bx = (velocityX[Index2(i + 1, j)] - velocityX[Index2(i, j)]) / grid.getCellDx().x;
				double by = (velocityY[Index2(i, j + 1)] - velocityY[Index2(i, j)]) / grid.getCellDx().y;
				b[pressure.getLinearIndex(i, j)] = -(Scene::kDensity / dt) * (bx + by);
			}
		}

		//Calculamos presiones usando el solver.


		for (int i = 0; i < pressure.getSize().x; i++)
		{
			for (int j = 0; j < pressure.getSize().y; j++)
			{
				//Primero, obtenemos el indice en una dimension para saber que fila rellenar
				int index = pressure.getLinearIndex(i, j);


				//Creamos una variable donde añadimos a cada valor de la diagonal 2/dx^2 + 2/dy^2 por defecto
				float elem = 2.0 / (grid.getCellDx().x * grid.getCellDx().x) + 2.0 / (grid.getCellDx().y * grid.getCellDx().y);

				//Izquierda
				if (i <= 0) //Si es borde izquierdo, le restamos a elem 1/dx^2
				{
					elem -= 1.0 / (grid.getCellDx().x * grid.getCellDx().x);
				}
				else //Si no es borde izquierdo, le ponemos como valor al de su izquierda -1/dx^2
				{
					//index - 1
					int new_index = pressure.getLinearIndex(i - 1, j);
					A.add_to_element(index, new_index, -1.0 / (grid.getCellDx().x * grid.getCellDx().x));
				}


				//Derecha
				if (i + 1 >= grid.getSize().x) //Si es borde derecho, le restamos a elem 1/dx^2
				{
					elem -= 1.0 / (grid.getCellDx().x * grid.getCellDx().x);
				}
				else //Si no es borde derecho, le ponemos como valor al de su derecha -1/dx^2
				{
					//index + 1
					int new_index = pressure.getLinearIndex(i + 1, j);
					A.add_to_element(index, new_index, -1.0 / (grid.getCellDx().x * grid.getCellDx().x));
				}
				//Abajo
				if (j <= 0) //Si es borde de abajo, le restamos a elem 1/dy^2
				{
					elem -= 1.0 / (grid.getCellDx().y * grid.getCellDx().y);
				}
				else //Si no es borde de abajo, le ponemos como valor al de abajo -1/dy^2
				{
					int new_index = pressure.getLinearIndex(i, j - 1);
					A.add_to_element(index, new_index, -1.0 / (grid.getCellDx().y * grid.getCellDx().y));
				}
				//Arriba
				if (j + 1 < grid.getSize().y) //Si no es borde de arriba se le pone de valor al de arriba -1/dy^2
				{
					int new_index = pressure.getLinearIndex(i, j + 1);
					A.add_to_element(index, new_index, -1.0 / (grid.getCellDx().y * grid.getCellDx().y));
				}
				//Si es borde no se hace nada porque es abierto


				//Rellenamos la diagonal con el valor resultante
				A.add_to_element(index, index, elem);
			}
		}

		//Solver para obtener p
		double  residual_out = 0.0;
		int iterations_out = 0.0;
		solver.solve(A, b, p, residual_out, iterations_out);


		//Añadir a la matriz pressure el valor del vector p utilizando getLinearIndex
		for (int i = 0; i < pressure.getSize().x; i++)
		{
			for (int j = 0; j < pressure.getSize().y; j++)
			{
				int index = pressure.getLinearIndex(i, j);
				pressure[Index2(i, j)] = p[index];
			}
		}

		//Después de calcular la presión mediante el solver obtenemos las velocidades:


		//Velocidades en X (Sin tocar los bordes izquierdo y derecho)
		for (int i = 1; i < velocityX.getSize().x - 1; i++)
		{
			for (int j = 0; j < velocityX.getSize().y; j++)
			{
				velocityX[Index2(i, j)] += -(dt / Scene::kDensity) * ((pressure[Index2(i, j)] - pressure[Index2(i - 1, j)]) / grid.getCellDx().x);
			}
		}

		//Velocidades en Y (Sin tocar bordes arriba y abajo)
		for (int i = 0; i < velocityY.getSize().x; i++)
		{
			for (int j = 1; j < velocityY.getSize().y - 1; j++)
			{
				velocityY[Index2(i, j)] += -(dt / Scene::kDensity) * ((pressure[Index2(i, j)] - pressure[Index2(i, j - 1)]) / grid.getCellDx().y);
			}
			//OH NO nuevo ataque del perro devora galletas aaaaaaaaaaaa
		}

		//Velocidades en el borde superior
		for (int i = 0; i < velocityY.getSize().x; i++)
		{
			velocityY[Index2(i, velocityY.getSize().y - 1)] += -(dt / Scene::kDensity) * ((-pressure[Index2(i, velocityY.getSize().y - 2)]) / grid.getCellDx().y);

		}

	}
}