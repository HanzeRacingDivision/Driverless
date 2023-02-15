import plotly.express as px
import pandas as pd
import argparse

def main(file_name):
    df = pd.read_json(file_name, lines=True)

    fig = px.scatter_geo(df,lat='lat',lon='lon', hover_name="time_stamp")
    fig.update_layout(title = 'World map', title_x=0.5)
    fig.show()

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser()
    parser.add_argument('log_file', help='log file to process')
    args = parser.parse_args()
    main(args.log_file)
        