from typing import Dict
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
GOOGLE_MAPS_API_KEY = "AIzaSyDlDzM_4vM_1tBMuHmB-0Qnba73nVzLlrM"
import requests
import creds
from sklearn.mixture import GaussianMixture
import numpy as np

class GoogleMapsAPI:
    def __init__(self, api_key):
        self.api_key = api_key

    def get_distance_matrix(self, origins, destinations, mode, departure_time):
        base_url = "https://maps.googleapis.com/maps/api/distancematrix/json"
        params = {
            "origins": "|".join(origins),
            "destinations": "|".join(destinations),
            "mode": mode,
            "departure_time": departure_time,
            "key": self.api_key,
        }

        response = requests.get(base_url, params=params)
        result = response.json()
        return result


class RouteOptimizer:
    def optimize_route(self) -> Dict[str, any]:
        """
        Optimizes the travel route using the Travelling Salesman Problem approach.
        :return: A dictionary containing the optimized route.
        """
        self.google_maps_api = GoogleMapsAPI(api_key=creds.GOOGLE_MAPS_API_KEY)
        distance_matrix = self.google_maps_api.get_distance_matrix(self.destinations, self.transportation_mode, self.departure_time)

        # Check for any origin with ZERO_RESULTS for all routes
        for idx, row in enumerate(distance_matrix["rows"]):
            if all(element.get("status") == "ZERO_RESULTS" for element in row["elements"]):
                return {
                    "error": f"No route can be found from '{self.destinations[idx]}' to any other destinations."
                }
        
        data = self.create_data_model(distance_matrix)
        
        # Create the routing index manager.
        manager = pywrapcp.RoutingIndexManager(
            len(data["distance_matrix"]), data["num_vehicles"], data["depot"]
        )

        # Create Routing Model.
        routing = pywrapcp.RoutingModel(manager)

        def distance_callback(from_index, to_index):
            return data["distance_matrix"][manager.IndexToNode(from_index)][manager.IndexToNode(to_index)]
        
        transit_callback_index = routing.RegisterTransitCallback(distance_callback)

        # Define cost of each arc
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        # Setting first solution
        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC)

        # Solve the problem
        solution = routing.SolveWithParameters(search_parameters)

        # Get the optimized route and travel times
        index = routing.Start(0)
        route = [self.destinations[manager.IndexToNode(index)]]
        travel_times = []
        while not routing.IsEnd(index):
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route.append(self.destinations[manager.IndexToNode(index)])
            from_index = manager.IndexToNode(previous_index)
            to_index = manager.IndexToNode(index)
            if "duration" in distance_matrix["rows"][from_index]["elements"][to_index]:
                travel_times.append(
                    distance_matrix["rows"][from_index]["elements"][to_index][
                        "duration"
                    ]["text"]
                )

        return {"route": route, "travel_times": travel_times}
    
    def cluster_destination_gmm(self, n_clusters):
        """
        Cluster the given destinations using Gaussian Mixture Model.
        :param n_clusters: Number of clusters to form.
        :return: Cluster labels for each destination.
        """
        # Convert destination names to lat-lon coordinates
        destinations_coords = []
        for destination in self.destinations:
            lat_lng = self.google_maps_api.get_lat_lng(destination)
            lat_lng["lon"] = lat_lng.pop("lng")

        # Extract latidues and longitudes from the coordinates
        data = np.array([d["lat"], d["lon"]] for d in destinations_coords)

        # Create a Gaussion Mixture Model with the specified number of components(cluster)
        gmm = GaussianMixture(n_components=n_clusters,
                              init_params="k-means++",
                              covariance_type="full",
                              reg_covar=1e-06,
                              random_state=0,
                              )
        
        # Fit the GMM to the data
        gmm.fit(data)

        # Predict the cluster for each destination
        labels = gmm.predict(data)

        return labels

