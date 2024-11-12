#include "Grafo.hxx"
#include <fstream>
#include <sstream>
#include <string>
#include <queue>
#include <limits>
#include <iostream>
#include <cfloat>
#include <set>

template <class T>
Grafo<T>::Grafo() {
    this->aristas = NULL;
}

template <class T>
void Grafo<T>::setVertices(std::vector<T> vertices) {
    this->vertices = vertices;
}

template <class T>
void Grafo<T>::setAristas(double** aristas) {
    this->aristas = aristas;
}

template <class T>
const std::vector<T>& Grafo<T>::getVertices() const {
    return this->vertices;
}

template <class T>
double** Grafo<T>::getAristas() {
    return this->aristas;
}

template <class T>
T* Grafo<T>::getVertice(int indice){
    if(indice<= cantVertices())
    return &vertices[indice];
    else
    return nullptr;
}

template <class T>
int Grafo<T>::cantVertices() {
    return vertices.size();
}

template <class T>
int Grafo<T>::cantAristas() {
    int suma = 0;
    for (int i = 0; i < cantVertices(); i++) {
        for (int j = 0; j < cantVertices(); j++) {
            if (*(*(aristas + i) + j) != 0) suma++;
        }
    }
    return suma;
}

template <class T>
int Grafo<T>::buscarVertice(T ver) {
    int ind = -1;
    for (int i = 0; i < cantVertices(); i++) {
        if (vertices[i] == ver) ind = i;
    }
    return ind;
}

template <class T>
bool Grafo<T>::insertarVertice(T ver) {
    bool res = false;
    if (buscarVertice(ver) == -1) {
        vertices.push_back(ver);
        double** nmatriz = new double*[cantVertices()];
        for (int i = 0; i < cantVertices(); i++) {
            *(nmatriz + i) = new double[cantVertices()];
        }
        for (int i = 0; i < cantVertices() - 1; i++) {
            for (int j = 0; j < cantVertices() - 1; j++) {
                *(*(nmatriz + i) + j) = *(*(aristas + i) + j);
            }
        }

        // Inicializar la nueva fila (para el nuevo vértice)
        for (int j = 0; j < cantVertices(); j++) {
            nmatriz[cantVertices() - 1][j] = 0; // Inicializa todos los elementos de la nueva fila
        }
        // Inicializar la nueva columna (para el nuevo vértice)
        for (int i = 0; i < cantVertices(); i++) {
            nmatriz[i][cantVertices() - 1] = 0; // Inicializa todos los elementos de la nueva columna
        }

        for (int i = 0; i < cantVertices() - 1; i++) {
            delete[] *(aristas + i);
        }
        delete[] aristas;
        aristas = nmatriz;
        res = true;
    }
    return res;
}

template <class T>
bool Grafo<T>::insertarArista(int i_ori, int i_des, float cos) {
    bool res = false;
    if (i_ori != -1 && i_des != -1) {
        if (*(*(aristas + i_ori) + i_des) == 0) {
            *(*(aristas + i_ori) + i_des) = cos;
            res = true;
        }
    }
    return res;
}

template <class T>
bool Grafo<T>::insAristaNoDir(T ori, T des, int cos) {
    bool res1 = insertarArista(ori, des, cos);
    bool res2 = insertarArista(des, ori, cos);
    return (res1 && res2);
}

template <class T>
int Grafo<T>::buscarArista(T origen, T destino) {
    int res = -1;
    int i_ori = buscarVertice(origen);
    int i_des = buscarVertice(destino);
    if (i_ori != -1 && i_des != -1) {
        res = *(*(aristas + i_ori) + i_des);
    }
    return res;
}

template <class T>
bool Grafo<T>::eliminarVertice(T ver) {
    int i_ori = buscarVertice(ver);
    int numVer = cantVertices();

    bool eliminado = false;

    if (i_ori != -1) {
        eliminado = true;

        // Eliminar el vértice del vector de vértices
        vertices.erase(vertices.begin() + i_ori);

        // Eliminar fila de la matriz de adyacencia
        delete[] this->aristas[i_ori];

        // Mover las filas restantes para sobreescribir la fila eliminada
        for (int i = i_ori; i < numVer - 1; ++i) {
            aristas[i] = aristas[i + 1];
        }

        // Eliminar la columna en la matriz de adyacencia
        for (int i = 0; i < numVer - 1; i++) {
            for (int j = i_ori; j < numVer - 1; j++) {
                aristas[i][j] = aristas[i][j + 1];
            }
        }

        // Redimensionar las filas restantes
        for (int i = 0; i < numVer - 1; i++) {
            int* nuevafila = new int[numVer - 1];
            for (int j = 0; j < numVer - 1; j++) {
                nuevafila[j] = aristas[i][j];
            }

            // Liberar mem de fila antigua
            delete[] aristas[i];
            aristas[i] = nuevafila;
        }

        // Actualizar el número de vértices
        numVer--;
    }

    return eliminado;
}

template <class T>
bool Grafo<T>::eliminarArista(T origen, T destino) {
    bool res = false;
    int i_ori = buscarVertice(origen);
    int i_des = buscarVertice(destino);
    if (i_ori != -1 && i_des != -1) {
        *(*(aristas + i_ori) + i_des) = 0;
        res = true;
    }
    return res;
}

template <class T>
bool Grafo<T>::elimAristaNoDir(T origen, T destino) {
    bool res1 = eliminarArista(origen, destino);
    bool res2 = eliminarArista(destino, origen);

    return (res1 && res2);
}

template <class T>
std::vector<T> Grafo<T>::vecinosVertice(T ver) {
    int indice = buscarVertice(ver);
    std::vector<T> ver_vecinos;

    if (indice != -1) {
        // Ubicar los vecinos que le correspondan al vértice
        for (int i = 0; i < cantVertices(); i++) {
            if (aristas[indice][i] != 0) {
                ver_vecinos.push_back(vertices[i]);
            }
        }
    }

    // Ordenar los vecinos dependiendo de su tipo de dato
    std::sort(ver_vecinos.begin(), ver_vecinos.end());
    return ver_vecinos;
}

template <class T>
std::vector<unsigned long> Grafo<T>::indicesVecinos(unsigned long id){
    std::vector <unsigned long> vecinos ;
    for(int i=0; i<cantVertices(); i++){
        if(aristas[id][i]!=0){
            vecinos.push_back(i);
        }
    }


    return vecinos;
}

template <class T>
void Grafo<T>::plano() {
    typename std::vector<T>::iterator it = vertices.begin();
    for (; it != vertices.end(); it++) {
        std::cout << *it << " ";
    }
    std::cout << std::endl;
}

template <class T>
std::vector<T> Grafo<T>::DFS(T ver_inicial) {
    std::vector<bool> ver_visitados;
    std::vector<T> caminoDFS;
    std::stack<T> pila_ver;

    if (buscarVertice(ver_inicial) == -1) {
        std::cout << "El vértice " << ver_inicial << " no está dentro del grafo" << std::endl;
        return caminoDFS;
    }

    pila_ver.push(ver_inicial);

    ver_visitados.resize(cantVertices(), false);

    while (!pila_ver.empty()) {
        T ver_actual = pila_ver.top();
        pila_ver.pop();
        // Obtener todos los vértices que tengan a ver_actual como origen
        std::vector<T> ver_vecinos = vecinosVertice(ver_actual);
        int ind = buscarVertice(ver_actual);
        // Revisar que el vértice aún no ha sido visitado
        if (!ver_visitados[ind]) {
            std::cout << vertices[ind] << " ";
            ver_visitados[ind] = true;
            caminoDFS.push_back(vertices[ind]);

            typename std::vector<T>::reverse_iterator it = ver_vecinos.rbegin();
            for (; it != ver_vecinos.rend(); it++) {
                pila_ver.push(*it);
            }
        }
    }

    std::cout << std::endl;

    return caminoDFS;
}

template <class T>
std::vector<T> Grafo<T>::BFS(T ver_inicial) {
    std::vector<bool> ver_visitados;
    std::vector<T> caminoBFS;
    std::queue<T> pila_ver;

    if (buscarVertice(ver_inicial) == -1) {
        std::cout << "El vértice " << ver_inicial << " no está dentro del grafo" << std::endl;
        return caminoBFS;
    }

    pila_ver.push(ver_inicial);

    ver_visitados.resize(cantVertices(), false);

    while (!pila_ver.empty()) {
        T ver_actual = pila_ver.front();
        pila_ver.pop();
        // Obtener todos los vértices que tengan a ver_actual como origen
        std::vector<T> ver_vecinos = vecinosVertice(ver_actual);
        int ind = buscarVertice(ver_actual);
        // Revisar que el vértice aún no ha sido visitado
        if (!ver_visitados[ind]) {
            std::cout << vertices[ind] << " ";
            ver_visitados[ind] = true;
            caminoBFS.push_back(vertices[ind]);

            typename std::vector<T>::iterator it = ver_vecinos.begin();
            for (; it != ver_vecinos.end(); it++) {
                pila_ver.push(*it);
            }
        }
    }

    std::cout << std::endl;

    return caminoBFS;
}

// Dijkstra's Algorithm
template <class T>
std::vector<std::vector<unsigned long> >Grafo<T>::dijkstra(unsigned long i_fuente) {
    int numVertices = cantVertices();
    std::vector<float> dist(numVertices, FLT_MAX);
    std:: vector<unsigned long> pred(numVertices, -1);
    std::vector<unsigned long> q;

    std::vector< std::vector<unsigned long> > todasRutas;
    std::vector<unsigned long> rutita;
    std::vector<unsigned long> rutitaInversa;
    std::vector<unsigned long> vecinos;



    for(int i=0; i< cantVertices(); i++){
        q.push_back(i);
    }

    pred[i_fuente] = i_fuente;
    dist[i_fuente] = 0;
    
    while (!q.empty()) {
        //Algoritmo de que vertice analizar 
        unsigned long menor= q[0];
        unsigned long borrar=0;
        for(unsigned long i=0; i<cantVertices(); i++ ){
            for(int j=0; j<q.size(); j++ ){
                if (i==q[j]){
                    if(dist[i]<dist[menor]){
                        menor=q[j];
                        borrar=j;
                    }
                }
            }
        }

        q.erase(q.begin()+borrar);

        // fin algotirmo de que sacar 

        vecinos= indicesVecinos(menor);
        for (int i = 0; i < vecinos.size(); ++i) {
            int peso = aristas[menor][vecinos[i]];
            if (dist[menor] + peso < dist[vecinos[i]]) {
                dist[vecinos[i]] = dist[menor] + peso;
                pred[vecinos[i]] = menor;
            }
        }
    }

    //Construir rutas
    int predecesor;
    for(int i=0 ; i<cantVertices(); i++){
        predecesor= pred[i];
        if(!(predecesor ==-1)){
            if(i!=i_fuente)
            rutitaInversa.push_back(i);
            rutitaInversa.push_back((unsigned long) predecesor);
            while(!(predecesor==0) ){
                predecesor= pred[predecesor];
                rutitaInversa.push_back(predecesor);
            }
            for(int j=rutitaInversa.size()-1; j>-1; j--){
                rutita.push_back(rutitaInversa[j]);
            }
            todasRutas.push_back(rutita);
        }
        
        rutita.clear();
        rutitaInversa.clear();
    }

    return todasRutas;

    //for (int i = 0; i < numVertices; ++i) {
        //std::cout << "Distancia desde el nodo " << fuente << " al nodo " << vertices[i] << " es " << dist[i] << std::endl;
    //}
}










template <class T>
void Grafo<T>::encontrarPuentes() {
    int numVertices = cantVertices();
    std::vector<int> tiempos(numVertices, -1);
    std::vector<int> menorTiempo(numVertices, -1);
    std::vector<int> nodoPadre(numVertices, -1);
    int tiempo = 0;

    for (int i = 0; i < numVertices; i++) {
        if (tiempos[i] == -1) {
            std::stack<T> pila_ver;
            pila_ver.push(vertices[i]);
            std::vector<bool> ver_visitados(cantVertices(), false);

            while (!pila_ver.empty()) {
                T ver_actual = pila_ver.top();
                int ind = buscarVertice(ver_actual);
                pila_ver.pop();

                if (tiempos[ind] == -1) {
                    tiempos[ind] = menorTiempo[ind] = ++tiempo;
                    ver_visitados[ind] = true;
                    std::vector<T> ver_vecinos = vecinosVertice(ver_actual);

                    for (auto vecino : ver_vecinos) {
                        int v = buscarVertice(vecino);
                        if (tiempos[v] == -1) {
                            nodoPadre[v] = ind;
                            pila_ver.push(vertices[ind]);
                            pila_ver.push(vecino);
                        } else if (v != nodoPadre[ind]) {
                            menorTiempo[ind] = std::min(menorTiempo[ind], tiempos[v]); //encuentra el menor entre ambos
                        }
                    }
                } else {
                    int v = nodoPadre[ind];
                    if (v != -1) {
                        menorTiempo[v] = std::min(menorTiempo[v], menorTiempo[ind]);
                        if (menorTiempo[ind] > tiempos[v]) {
                            std::cout << "Puente encontrado: " << vertices[v] << " - " << vertices[ind] << std::endl;
                        }
                    }
                }
            }
        }
    }
}


template <class T>
bool Grafo<T>::esEuler() {
    int numVertices = cantVertices();
    int  imparCont= 0;

    for (int i = 0; i < numVertices; ++i) {
        int grado = 0;
        for (int j = 0; j < numVertices; ++j) {
            if (aristas[i][j] != 0) {
                grado++;
            }
        }
        if (grado % 2 != 0) {
            imparCont++;
        }
    }

    if (imparCont == 0) {
        return true;
    } else if (imparCont == 2) {
        return true;
    } else {
        return false;
    }
}

template <class T>
 std::vector<int> Grafo<T>::circuitoEuler() {
    std::vector<int> recorridoEuler = NULL;
    if (!esEuler()) {
        return recorridoEuler;
    }
    std::stack<int> pila;
    int numVertices = cantVertices();
    int inicio = 0;

    for (int i = 0; i < numVertices; ++i) {
        int grado = 0;
        for (int j = 0; j < numVertices; ++j) {
            if (aristas[i][j] != 0) {
                grado++;
            }
        }
        if (grado % 2 != 0) {
            inicio = i;
            break;
        }
    }

    pila.push(inicio);
    while (!pila.empty()) {
        int u = pila.top();
        int v;
        for (v = 0; v < numVertices; ++v) {
            if (aristas[u][v] != 0) {
                break;
            }
        }
        if (v == numVertices) {
            recorridoEuler.push_back(u);
            pila.pop();
        } else {
            pila.push(v);
            aristas[u][v] = 0;
            aristas[v][u] = 0; // Since the graph is undirected
        }
    }
    return recorridoEuler;
    /*std::cout << "Camino o Circuito de Euler: ";
    for (int i = 0; i < recorridoEuler.size(); ++i) {
        std::cout << vertices[recorridoEuler[i]] << " ";
    }
    std::cout << std::endl;
    */
}

template <class T>
void Grafo<T>::floydWarshall() {
    int n = cantVertices();
    std::vector<std::vector<int>> D(n, std::vector<int>(n, 900));
    std::vector<std::vector<int>> P(n, std::vector<int>(n, -1));

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (i == j) {
                D[i][j] = 0;
                P[i][j] = -1;
            } else if (aristas[i][j] != 0) {
                D[i][j] = aristas[i][j];
                P[i][j] = i;
            } else {
                D[i][j] = 900;
                P[i][j] = -1;
            }
        }
    }

    for (int k = 0; k < n; ++k) {
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                if (D[i][k] != 900 && D[k][j] != 900 && D[i][k] + D[k][j] < D[i][j]) {
                    D[i][j] = D[i][k] + D[k][j];
                    P[i][j] = P[k][j];
                }
            }
        }
    }
    /*
    std::cout << "Matriz de distancias mínimas: " << std::endl;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (D[i][j] == 999999) {
                std::cout << "INF" << " ";
            } else {
                std::cout << D[i][j] << " ";
            }
        }
        std::cout << std::endl;
    }

    std::cout << "Matriz de predecesores: " << std::endl;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (P[i][j] == -1) {
                std::cout << "/" << " ";
            } else {
                std::cout << P[i][j] << " ";
            }
        }
        std::cout << std::endl;
    }
    */
}
template <class T>
void Grafo<T>::kruskal() {
    int numVertices = cantVertices();
    std::vector<std::pair<int, std::pair<int, int>>> aristasGrafo; // Almacenar las aristas como (peso, (origen, destino))
    std::vector<std::pair<int, int>> arbol;


    std::vector<int> bosque(numVertices);
    for (int i = 0; i < numVertices; ++i) {
        bosque[i] = i;
    }

    for (int i = 0; i < numVertices; ++i) {
        for (int j = i + 1; j < numVertices; ++j) {
            if (aristas[i][j] != 0) {
                aristasGrafo.push_back({aristas[i][j], {i, j}});
            }
        }
    }

    // Ordenar las aristas por peso
    std::sort(aristasGrafo.begin(), aristasGrafo.end());

    auto encontrarConjunto = [&](int v) {
        if (bosque[v] != v) {
            bosque[v] = encontrarConjunto(bosque[v]);
        }
        return bosque[v];
    };

    // Función para unir dos conjuntos
    auto unirConjuntos = [&](int u, int v) {
        int raizU = encontrarConjunto(u);
        int raizV = encontrarConjunto(v);
        bosque[raizU] = raizV; // Unir ambos conjuntos
    };

    for (size_t i = 0; i < aristasGrafo.size(); ++i) {
        int peso = aristasGrafo[i].first;
        int u = aristasGrafo[i].second.first;
        int v = aristasGrafo[i].second.second;

        int set_u = encontrarConjunto(u);
        int set_v = encontrarConjunto(v);

        // Si la arista conecta dos árboles diferentes, añadirla al árbol y unir los conjuntos
        if (set_u != set_v) {
            arbol.push_back({u, v});
            unirConjuntos(set_u, set_v);
        }
    }
    /*
    // Imprimir el árbol de recubrimiento mínimo
    std::cout << "Aristas en el árbol de recubrimiento mínimo:" << std::endl;
    for (size_t i = 0; i < arbol.size(); ++i) {
        std::cout << "Origen: " << arbol[i].first << ", Destino: " << arbol[i].second << std::endl;
    }
    */
}


template <class T>
std::vector<std::vector<unsigned long >> Grafo<T>::prim(int indiceInicio) {
    int numVertices = cantVertices();
    std::vector<bool> enArbol(numVertices, false);  // Marca si el vértice está en el árbol
    std::vector<int> minPeso(numVertices, std::numeric_limits<int>::max());  // Peso mínimo
    std::vector<int> padre(numVertices, -1);  // Almacena el vértice padre de cada vértice

    // Obtener el índice del vértice de inicio
    T *TInicio = getVertice(indiceInicio);
    if (TInicio== nullptr) {
        std::cout << "El vértice " << indiceInicio << " no está en el grafo." << std::endl;
        return {};
    }

    minPeso[indiceInicio] = 0;
    std::vector<std::vector<unsigned long>> todasRutas(numVertices);  // Almacena todas las rutas

    // Conjunto de aristas para manejar vecinos inmediatos sin duplicados
    std::set<int> vecinosPendientes;  
    vecinosPendientes.insert(indiceInicio);

    // Bucle principal de Prim para construir el árbol de expansión mínima
    while (!vecinosPendientes.empty()) {
        int u = -1;
        int pesoMinimo = std::numeric_limits<int>::max();

        // Encontrar el vértice `u` con el menor peso de conexión no incluido en el árbol
        for (int v : vecinosPendientes) {
            if (minPeso[v] < pesoMinimo) {
                pesoMinimo = minPeso[v];
                u = v;
            }
        }

        // Si no se encuentra un vértice, terminar
        if (u == -1) break;

        vecinosPendientes.erase(u);  // Eliminar `u` del conjunto
        enArbol[u] = true;           // Marcar `u` como parte del árbol

        // Construir la ruta desde el inicio hasta el vértice `u` en `todasRutas`
        std::vector<unsigned long> ruta;
        int actual = u;
        while (actual != -1) {
            ruta.push_back(actual);
            actual = padre[actual];
        }
        std::reverse(ruta.begin(), ruta.end());
        todasRutas[u] = ruta;

        // Explorar vecinos de `u` y actualizar pesos mínimos
        std::vector<unsigned long> vecinos = indicesVecinos(u);
        for (int i = 0; i < vecinos.size(); ++i) {
            int v = vecinos[i];
            int pesoArista = static_cast<int>(aristas[u][v]);
            if (!enArbol[v] && pesoArista < minPeso[v]) {
                minPeso[v] = pesoArista;
                padre[v] = u;
                vecinosPendientes.insert(v);
            }
        }
    }

    // Imprimir el árbol de expansión mínima
   /*  std::cout << "Árbol de expansión mínima de Prim:" << std::endl;
    for (int i = 0; i < numVertices; ++i) {
        if (padre[i] != -1) {
            std::cout << "Arista: " << vertices[padre[i]] << " - " << vertices[i] 
                      << " con peso " << aristas[padre[i]][i] << std::endl;
        }
    } */

    return todasRutas;
}

template <class T>
double Grafo<T>::obtenerCosto(int id1, int id2){
    return aristas[id1][id2];
}
