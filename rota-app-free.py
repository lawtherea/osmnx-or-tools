import osmnx as ox
import networkx as nx
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import matplotlib.pyplot as plt

# --- 1. Lista de endereços (ou nomes de locais) ---
enderecos = [
    "Avenida Paulista, 1000, São Paulo, Brasil",
    "Rua Augusta, 1500, São Paulo, Brasil",
    "Mercado Municipal de São Paulo",
    "Praça da Sé - São Paulo",
    "Aeroporto de Congonhas"
]

# --- 2. Geocodifica (só para fins locais, proibido uso comercial) ---
coordenadas = [ox.geocode(endereco) for endereco in enderecos]

# --- 3. Baixa o grafo com ruas em torno de todos os pontos ---
media_lat = sum(lat for lat, lon in coordenadas) / len(coordenadas)
media_lon = sum(lon for lat, lon in coordenadas) / len(coordenadas)
G = ox.graph_from_point((media_lat, media_lon), dist=8000, network_type='drive')

# --- 4. Mapeia coordenadas para nós do grafo ---
nos = [ox.distance.nearest_nodes(G, lon, lat) for lat, lon in coordenadas]

# --- 5. Calcula a matriz de distâncias reais (em metros) ---
n = len(nos)
matriz = [[0]*n for _ in range(n)]
for i in range(n):
    for j in range(n):
        if i != j:
            matriz[i][j] = int(nx.shortest_path_length(G, nos[i], nos[j], weight='length'))

# --- 6. Resolve o TSP com OR-Tools ---
def resolver_tsp(matriz):
    manager = pywrapcp.RoutingIndexManager(len(matriz), 1, 0)  # 1 veículo, início em 0
    routing = pywrapcp.RoutingModel(manager)

    def callback(from_index, to_index):
        return matriz[manager.IndexToNode(from_index)][manager.IndexToNode(to_index)]

    transit_callback_index = routing.RegisterTransitCallback(callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Configura estratégia de solução inicial
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    solution = routing.SolveWithParameters(search_parameters)

    if solution:
        index = routing.Start(0)
        rota = []
        while not routing.IsEnd(index):
            rota.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))
        rota.append(rota[0])  # volta ao início
        return rota
    else:
        return None

# --- 7. Executa e mostra resultado ---
rota_otimizada = resolver_tsp(matriz)

# Converte os índices da rota otimizada em coordenadas
rota_coordenadas = [coordenadas[i] for i in rota_otimizada]

if rota_otimizada:
    print("\n--- ROTA OTIMIZADA ---")
    for i in rota_otimizada:
        print(f"{i}: {enderecos[i]}")
    print("\n--- GEOCODIFICAÇAO (O QUE O FRONT RECEBERÁ) ---")
    for i, (lat, lon) in enumerate(rota_coordenadas):
        print(f"{i+1}. ({lat}, {lon})")
else:
    print("Não foi possível resolver o TSP.")

# --- 8. Plota a rota no mapa ---
if rota_otimizada:
    rota_nos = [nos[i] for i in rota_otimizada]
    rota_final = []
    for i in range(len(rota_nos) - 1):
        path = nx.shortest_path(G, rota_nos[i], rota_nos[i + 1], weight='length')
        if rota_final and path[0] == rota_final[-1]:
            path = path[1:]
        rota_final += path

    # Calcula distância total
    distancia_total_metros = 0
    for u, v in zip(rota_final[:-1], rota_final[1:]):
        dados_aresta = G.get_edge_data(u, v)
        if dados_aresta:
            menor = min(d['length'] for d in dados_aresta.values())
            distancia_total_metros += menor
    distancia_total_km = distancia_total_metros / 1000
    print(f"\n🚗 Distância total percorrida: {distancia_total_km:.2f} km")

    # Plota o grafo com a rota
    fig, ax = ox.plot_graph_route(
        G,
        rota_final,
        route_linewidth=4,
        node_size=0,
        route_color="green",
        bgcolor="white",
        show=False,
        close=False,
    )

    # Coordenadas dos nós da rota otimizada
    coords = [(G.nodes[n]['x'], G.nodes[n]['y']) for n in rota_nos]

    # Ponto inicial (origem/fim) em vermelho sem texto
    x0, y0 = coords[0]
    ax.scatter(x0, y0, c="red", s=120, zorder=5, label="Início/Fim")

    # Pontos intermediários com número baseado no índice original
    for i in range(1, len(rota_nos) - 1):  # exclui início/fim
        node_id = rota_nos[i]
        endereco_index = rota_otimizada[i]
        x = G.nodes[node_id]['x']
        y = G.nodes[node_id]['y']

        ax.scatter(x, y, c="blue", s=80, zorder=4)
        texto = f"{endereco_index}"
        ax.annotate(
            texto,
            xy=(x, y),
            xytext=(3, 3),
            textcoords="offset points",
            fontsize=9,
            fontweight='bold',
            color="black",
            bbox=dict(boxstyle="circle,pad=0.3", fc="white", ec="gray", lw=0.8, alpha=0.9),
            zorder=6,
        )

    ax.legend()
    plt.show()


