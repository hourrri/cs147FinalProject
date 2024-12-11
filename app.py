from collections import defaultdict
from flask import Flask, json, render_template, request
from azure.eventhub import EventHubConsumerClient
from datetime import datetime
import threading

app = Flask(__name__)
current_goal = "Blue Button" # default goal name

CONNECTION_STR = "Endpoint=sb://info147.servicebus.windows.net/;SharedAccessKeyName=RootManageSharedAccessKey;SharedAccessKey=vHoZelz2DoLXHxzzeyjE4OkY/6tDD929w+AEhBoEGsY="
EVENT_HUB_NAME = "147event"

received_messages = []
processed_data = {}
parsed_data = defaultdict(dict)

def on_event(partition_context, event):
    """Callback to process received events."""
    message = event.body_as_str()
    timestamp = datetime.now().isoformat()

    try:
        data = json.loads(message)

        blue_button = data.get("blueButton", 0)
        reset_button = data.get("resetButton", 0)
        hit_goal = data.get("hitGoal", 0)
        num_of_pickups = data.get("numOfPickups", 0)
        temperature = data.get("tempurature", 0.00)
        bpm = data.get("BPM", 0)

        parsed_data[timestamp] = {
            "blueButton": blue_button,
            "resetButton": reset_button,
            "hitGoal": hit_goal,
            "numOfPickups": num_of_pickups,
            "temperature": temperature,
            "BPM": bpm,
        }

        print(f"Parsed data at {timestamp}: {parsed_data[timestamp]}")

        received_messages.append(message)

    except json.JSONDecodeError as e:
        print(f"Failed to parse message: {message}, Error: {e}")

    partition_context.update_checkpoint(event)

@app.route('/', methods=['GET', 'POST'])
def index():
    global current_goal
    if request.method == 'POST':
        # update the goal name from the form
        current_goal = request.form['goalName']
    
    # render the template and change to the new goal name
    return render_template("index.html", messages=parsed_data, goalName=current_goal)


def start_eventhub_listener():
    """Start the Event Hub listener in a background thread."""
    client = EventHubConsumerClient.from_connection_string(
        CONNECTION_STR, consumer_group="$Default", eventhub_name=EVENT_HUB_NAME
    )
    with client:
        client.receive(on_event=on_event, starting_position="-1")

if __name__ == '__main__':
    listener_thread = threading.Thread(target=start_eventhub_listener, daemon=True)
    listener_thread.start()
    app.run(debug=True, host='0.0.0.0', port=500)
