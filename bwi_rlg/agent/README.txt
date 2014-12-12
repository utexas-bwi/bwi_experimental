Make a copy of the parsing information that the agent can overwrite as needed:

$cp -r spf/geoquery/experiments/template/asp_mods/ spf/geoquery/experiments/template/dialog_writeable/

Now the system should be ready to run. If this is the very first run, build the parser in the new directory with

$python dialog/main.py dialog/ retrain -restart_parser

For a user with id [USER_ID] (used to separate multiple simultaneous dialogues), get the next system command with

$python dialog/main.py dialog/ offline [USER_ID]

The system will write its response to 'offline_data/outputs/[USER_ID]_output.txt'

After the first call, the user will need to provide feedback before the agent can be invoked again. When invoked with

$python dialog/main.py dialog/ offline [USER_ID]

again, the system will read 'offline_data/inputs/[USER_ID]_input.txt' for a single line of user text; it will load its previous dialogue state and respond to this text, again writing to the aforementioned output file

When dialogue concludes, the ASP goal settled on by the system will be written to 'offline_data/commands/[USER_ID]_command.txt'
Negating this and appending it to an asp world text, clingo should be able to generate a plan to accomplish the user's goals

--- Temporary for AAMAS experiment

You can now exclude dialogs that led to 'testing' goals during retraining with

$python dialog/main.py dialog/ retrain -exclude_test_goals
