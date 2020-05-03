import socket
import sys
import threading
import math
import time
import json
import copy


class Router(object):
    def __init__(self, config_file, route_update_interval=30, update_interval=1):
        self.update_interval = update_interval
        self.route_update_interval = route_update_interval
        
        self.adj = {}
        self.ports_table = {}
        
        # record the last time that the neighbours send a hearbeat 
        self.hearbeat_record = {}
        # flag that if start to check hearbeat or not 
        self.start_hearbeat = False
        
        self.port = None
        self.id = None
        
        self.send_sock = None
        self.listen_sock = None
        
        self.init_config(config_file)
        self.init_socket()
        
    def init_socket(self):
        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.hearbeat_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        self.listen_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.listen_sock.bind(('127.0.0.1', self.port))
    
    def init_config(self, config_file):
        
        with open(config_file, 'r') as fp:
            line = fp.readline().strip()
            ls = line.split()
            self.id = ls[0]
            self.port = int(ls[1])
            
            #self.table[self.id] = {}
            self.adj[self.id] = {}
            self.ports_table[self.id] = self.port
            
            line = fp.readline().strip()
            for i in range(int(line)):
                line = fp.readline().strip()
                ls = line.split()
                
                # {id:(cost, port), }
                self.ports_table[ls[0]] = int(ls[2])
                
                self.hearbeat_record[ls[0]] = time.time()
                
                self.adj[self.id][ls[0]] = float(ls[1])
                if ls[0] not in self.adj:
                    self.adj[ls[0]] = {}
                self.adj[ls[0]][self.id] = float(ls[1])
         
    
    def send_broadcast(self):
        
        while True:
            # cast the dict to dict str
            str_table = json.dumps({'type': 'data', 'adj': self.adj, 'port': self.ports_table, 'id': self.id, 'route':[self.id]})
            # boardcast all neighbours
            for k, v in self.ports_table.items():
                if v == self.port:
                    continue
                self.send_sock.sendto(bytes(str_table, 'ascii'), ('127.0.0.1', v))
            
            # sleep the UPDATE_INTERVAL seconds
            time.sleep(self.update_interval)
    
    def send_hearbeat(self):
        while True:
            # cast the dict to dict str
            str_table = json.dumps({'type': 'hearbeat', 'port': self.ports_table, 'id': self.id})
            # boardcast all neighbours hear beat
            for k, v in self.ports_table.items():
                if v == self.port:
                    continue
                self.hearbeat_sock.sendto(bytes(str_table, 'ascii'), ('127.0.0.1', v))
            
            # sleep the UPDATE_INTERVAL*0.8 seconds (default 0.8 seconds)
            time.sleep(self.update_interval*0.8)    
            
    
    def update_table(self, other_data):
        
        # get all information of the packet data
        other_adj = other_data['adj']
        other_ports = other_data['port']
        other_id = other_data['id']
        other_route = other_data['route']
        
        # update the ports table
        for k, v in other_ports.items():
            self.ports_table[k] = v
        
        # update the adj table
        for k1, v1 in other_adj.items():
            if k1 not in self.adj:
                self.adj[k1] = {}
                
            for k2, v2 in v1.items():
                self.adj[k1][k2] = v2
                
                if k2 not in self.adj:
                    self.adj[k2] = {}
                
                self.adj[k2][k1] = v2
        
    
    def check_router_down(self):
        
        # do not start check hearbeat until pass self.route_update_interval seconds
        if not self.start_hearbeat:
            time.sleep(self.route_update_interval)
            self.start_hearbeat = True
        
        while True:
            for n in list(self.ports_table.keys()):
                if n == self.id:
                    continue
                if time.time() - self.hearbeat_record[n] > 3*self.update_interval*0.8:
                    self.remove_from_adjacency_table(n)
                    self.ports_table.pop(n)

            time.sleep(self.update_interval*0.8)
    
    def receive_broadcast(self):
        
        while True:
            
            # receive a packet
            data, addr = self.listen_sock.recvfrom(1024)
            # parse the packet
            parsed_data = json.loads(str(data, 'ascii'))
            other_type = parsed_data['type']
            other_id = parsed_data['id']
            
            if other_type == 'data':
                other_route = parsed_data['route']
            
                # boardcast all neighbours
                # excluding the router from which it received this link- state packet in the first place
                for k, v in self.ports_table.items():
                    # 'k in other route' for reduce such unnecessary broadcasts
                    if v == self.port or k in other_route:
                        continue
                    
                    if self.id not in parsed_data['route']:
                        parsed_data['route'].append(self.id)
                        
                    self.send_sock.sendto(bytes(json.dumps(parsed_data), 'ascii'), ('127.0.0.1', v))
                
                # update the neighbours table
                self.update_table(parsed_data)
            elif other_type == 'hearbeat':
                #self.check_router_down(other_id)
                self.hearbeat_record[other_id] = time.time()
            #print(self.adj)
            
            
    def make_adjacency_table(self):
        
        nodes = [k for k in self.ports_table]
        adj = {}
        for n in nodes:
            adj[n] = {}
            for nn in nodes:
                adj[n][nn] = math.inf
        
        for k, v in self.adj.items():
            for kk, vv in v.items():
                adj[k][kk] = vv
                adj[kk][k] = vv
        
        return adj
    
    
    def remove_from_adjacency_table(self, id):
        # if id in the adj, remove the id
        if id in self.adj:
            self.adj.pop(id)
            
        for k in self.adj.keys():
            if id in self.adj[k]:
                self.adj[k].pop(id)
                
                
    
    def dijkstra(self, start):
        
        visit = {k:0 for k in self.ports_table}
        adj = self.make_adjacency_table()
        dist = copy.deepcopy(adj[start])
        paths = {k:-1 for k in self.ports_table}
        
        nodes = self.ports_table.keys()
        
        for n in nodes:
            if adj[start][n] == math.inf:
                pass
            else:
                paths[n] = start
        
        for i in range(len(nodes)-1):
            min_cost = math.inf
            min_node = start
            
            for n in nodes:
                if visit[n] == 0 and dist[n] < min_cost:
                    min_node = n
                    min_cost = dist[n]

            visit[min_node] = 1
            
            for n in nodes:
                if visit[n] == 0 and dist[min_node] + adj[min_node][n] < dist[n]:
                    dist[n] = dist[min_node] + adj[min_node][n]
                    paths[n] = min_node
        
        
        return dist, paths
    
    
    def parse_paths(self, paths, start, end):
        path = []
        node = end
        path.append(node)
        while True :
            value = paths[node]
            node = value
            path.append(node)
            if value == start:
                break

        return path
    
    
    def calculate_shortest_path(self):
        
        while True:
            
            time.sleep(self.route_update_interval)
            
            dist, paths = self.dijkstra(self.id)
            print("I am Router {}".format(self.id))
            for n in self.ports_table:
                if n == self.id:
                    continue
                path = self.parse_paths(paths, self.id, n)
                print("Least cost path to router {}: {} and the cost is {:.1f}".format(n, ''.join(list(reversed(path))), dist[n]))
                
                
    def run(self):
        
        t1 = threading.Thread(target=self.send_broadcast, name='send broadcast')
        t2 = threading.Thread(target=self.receive_broadcast, name='receive broadcast')
        t3 = threading.Thread(target=self.calculate_shortest_path, name='calculate shortest path')
        
        t4 = threading.Thread(target=self.send_hearbeat, name='send hearbeat')
        t5 = threading.Thread(target=self.check_router_down, name='check router down')
        
        t1.start()
        t2.start()
        t3.start()
        t4.start()
        t5.start()
        
        t1.join()
        t2.join()
        t3.join()
        t4.join()
        t5.join()
        
        
        



if __name__ == "__main__":
    
    
    config_file = sys.argv[1]
    
    router = Router(config_file, route_update_interval=30, update_interval=1)
    
    '''    
    router.make_adjacency_table()
    
    print(router.adj)
    
    router.remove_from_adjacency_table('B')
    print(router.adj)'''
    '''router.send_broadcast()
    
    router.receive_broadcast()'''
    
    router.run()
