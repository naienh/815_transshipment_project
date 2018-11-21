import os, sys
from math import floor, ceil
import plotly
import plotly.graph_objs as go

def generate_network(model,location_data,timewindow,iftrans,filename):
    edge_text = dict(x=[],y=[],hovertext=[])
    edge_trace = go.Scatter(
        x=[],
        y=[],
        line=dict(width=1,color='#888'),
        hoverinfo='text',
        hovertext=[],
        mode='lines')

    for edge in model.A:
        counter = 0
        constructed_string = ['Arc: {} to {}'.format(*edge)]
        for k in model.K:
            if model.x[edge,k] == 1:
                counter += 1
                constructed_string.append('Vehicle {}:'.format(k))
                r_counter = 0
                for r in model.R:
                    if model.y[edge,k,r] == 1:
                        r_counter += 1
                        constructed_string.append('\tCarrying Order: {}, {} to {}'.format(r,model.p[r],model.d[r]))
                if r_counter == 0:
                    constructed_string.append('\tCarrying Nothing')

        node1, node2 = edge
        for _ in range(counter):
            x0, y0 = location_data[node1]
            x1, y1 = location_data[node2]
            scale_cofficient = 0.01*(-1)**_*ceil(_/2)
            edge_trace['x'] += tuple([x0+scale_cofficient, x1+scale_cofficient,None])
            edge_trace['y'] += tuple([y0+scale_cofficient, y1+scale_cofficient,None])
            edge_text['x'] += tuple([(1/3*x0+2/3*x1)])
            edge_text['y'] += tuple([(1/3*y0+2/3*y1)])
            edge_text['hovertext'] += tuple(['<br>'.join(constructed_string)])

    center_trace = go.Scatter(
        x=edge_text['x'],
        y=edge_text['y'],
        hoverinfo='text',
        hovertext=edge_text['hovertext'],
        mode='markers',
        marker=dict(
                symbol='circle',
                color='green',
                size=5,
    ))

    node_trace = go.Scatter(
        x=[],
        y=[],
        text=[],
        textfont=dict(color=[]),
        mode='markers+text',
        hoverinfo='x+y+text',
        hovertext=[],
        marker=dict(
            symbol=[],
            color=[],
            size=25,
            line=dict(width=[],color=[])),
    #     selected=dict(marker=dict(color='red',size=30))
    )

    for node in model.N:
        x, y = location_data[node]
        node_trace['x'] += tuple([x])
        node_trace['y'] += tuple([y])
        # construct doc string
        constructed_string = ['Node: {}'.format(node)]
        counter = 0
        for k in model.K:
            if node == model.o[k]:
                constructed_string.append('Initial Depot: Vehicle {}'.format(k))
                node_trace['marker']['symbol'] += tuple(['diamond'])
                node_trace['marker']['color'] += tuple(['lightcyan'])
                node_trace['marker']['line']['color'] += tuple(['purple'])
                node_trace['marker']['line']['width'] += tuple([1])
                node_trace['text'] += tuple(['{}'.format(k)])
                node_trace['textfont']['color'] += tuple(['black'])
                counter = 1
            if node == model.o_[k]:
                constructed_string.append('End Depot: Vehicle {}'.format(k))
                node_trace['marker']['symbol'] += tuple(['cross'])
                node_trace['marker']['color'] += tuple(['lightpink'])
                node_trace['marker']['line']['color'] += tuple(['purple'])
                node_trace['marker']['line']['width'] += tuple([1])
                node_trace['text'] += tuple(['{}'.format(k)])
                node_trace['textfont']['color'] += tuple(['black'])
                counter = 1
        if counter == 0:
            if node in model.T:
                # check if transshipment happens
                sum19 = [sum(model.y[j,node,k,r].value for j in model.N if (j,node) in model.A)\
                         + sum(model.y[node,j,l,r].value for j in model.N if (node,j) in model.A) == 2\
                         for r in model.R for k in model.K for l in model.K if l != k]
                if sum(sum19) >= 1:
                    node_trace['marker']['symbol'] += tuple(['circle'])
                    node_trace['marker']['color'] += tuple(['navy'])
                    node_trace['marker']['line']['color'] += tuple(['yellow'])
                    node_trace['marker']['line']['width'] += tuple([5])
                    node_trace['text'] += tuple([node])
                    node_trace['textfont']['color'] += tuple(['white'])
                    constructed_string.append('Transhipment Node')
                else:
                    node_trace['marker']['symbol'] += tuple(['circle'])
                    node_trace['marker']['color'] += tuple(['navy'])
                    node_trace['marker']['line']['color'] += tuple(['purple'])
                    node_trace['marker']['line']['width'] += tuple([1])
                    node_trace['text'] += tuple([node])
                    node_trace['textfont']['color'] += tuple(['white'])
            else:
                node_trace['marker']['symbol'] += tuple(['circle'])
                node_trace['marker']['color'] += tuple(['navy'])
                node_trace['marker']['line']['color'] += tuple(['purple'])
                node_trace['marker']['line']['width'] += tuple([1])
                node_trace['text'] += tuple([node])
                node_trace['textfont']['color'] += tuple(['white'])
        for r in model.R:
            if node == model.p[r]:
                constructed_string.append('Pick Up: Order {}'.format(r))
            if node == model.d[r]:
                constructed_string.append('Delivery: Order {}'.format(r))

        node_trace['hovertext'] += tuple(['<br>'.join(constructed_string)])

    arrow_dic = [dict(
                x=2/3*edge_trace['x'][3*i+1]+1/3*edge_trace['x'][3*i],
                y=2/3*edge_trace['y'][3*i+1]+1/3*edge_trace['y'][3*i],
                ax=1/3*edge_trace['x'][3*i+1]+2/3*edge_trace['x'][3*i],
                ay=1/3*edge_trace['y'][3*i+1]+2/3*edge_trace['y'][3*i],
                xref='x',
                yref='y',
                axref='x',
                ayref='y',
                showarrow=True,
                arrowhead=2,
                arrowsize=4,
                arrowwidth=0.5,
                arrowcolor='#888',
            ) for i in range(int(len(edge_trace['x'])/3))]
    timewindow_text = 'WITH Time Window' if timewindow else 'NO Time Window'
    trans_text = 'Allow Transshipment' if iftrans else 'NO Transshipment'
    title = 'Routing Case: {}<br>'.format(filename)+timewindow_text+'\t|\t'+trans_text

    fig = go.Figure(data=[edge_trace,node_trace,center_trace],
                 layout=go.Layout(
                    title=title,
                    titlefont=dict(size=16),
                    showlegend=False,
                    hovermode='closest',
                    margin=dict(b=20,l=5,r=5,t=40),
                    xaxis=dict(showgrid=True, zeroline=False, showticklabels=False, hoverformat='.2f'),
                    yaxis=dict(showgrid=True, zeroline=False, showticklabels=False, hoverformat='.2f'),
                    annotations = arrow_dic))

    return fig

class HiddenPrints(object):
    def __enter__(self):
        self._original_stdout = sys.stdout
        sys.stdout = None

    def __exit__(self, exc_type, exc_val, exc_tb):
        sys.stdout = self._original_stdout
