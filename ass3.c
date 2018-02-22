#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/wait.h>
#include <string.h>
#include <sys/types.h>

/*          Macros                          */
#define MAXPLAY 26
#define ROLLSIZE 6

/*          Structures                      */

typedef struct info {
    int playerNum;      //number of players in the game
    char* rollFile;     //string of roll filename
    int RFL;            //Roll File Length in number of rolls
    int CRN;            //Current Roll Number
    char rollStr[7];    //to store a roll
    char* ERRMes;       //message to send to stderr
    char* StLMes;       //message to send to player
    int playerList[26]; //contains the active players, 222 is a defective player
    int MesSize;        //for writing to player
    int attack;         //countains the attack of current round
    int MAX;            //is the maximum amount of points for a game
    int stLuciaCount;   //increments when a player stays in sL for a round
    int oldRound;       
    int newRound;
   // char* playerMes;
}info; 

typedef struct player {
    char rollStr[7];    //dice to send to players
    char reroll[7];     //dice to reroll
    int id;             //the player id; 0->25
    int active;         //1 if the player responded from their program
    char* playerName;   //A->Z
    char* playerType;   //from argv, links to EAIT,SCIENCE,HASS,HABS or MABS
    int playerMes;      //0-not current turn, 1-keepall, 2-reroll, 3-stay, 4-go
    int stLucia;        // 0 not in stLucia, 1 in stLucia
    int health;         //health of the player
    int tokens;         //tokens for the player
    int points;         //points for the player
    int IN[2];          //pipe linking to player stdin
    int OUT[2];         //pipe linking to player stdout
    int ERR[2];         //pipe linking to player stderr
    pid_t ppid;         //process identification   
} player;

/* possible dice rolls and playerName's */
const char roll[ROLLSIZE] = {'1','2','3','H','A','P'};
char* alpha[26] = {"A","B","C","D","E","F","G","H","I","J","K","L","M",
                        "N","O","P","Q","R","S","T","U","V","W","X","Y","Z"};

//      global array of struct type player
player player_struct[MAXPLAY];
/* to reference playername of player A:
    player_struct[0].playerName
*/


/*          Function declarations           */
void ERR(int status);
void check_file(char* filename, info* info_ptr);
void check_input(int argc, char*argv[], info* info_ptr);
void check_stderr(info* info_ptr, int id);
info* makeInfo(void);
void hubErr(int ext);
void makeplayer(info* info_ptr, char** argv, int argc);
void get_rolls(info* info_ptr, char* rollBuffer);
void arrangeP(int player);
char* arrange(char* rollStr);
void message(info* info_ptr, int cond, int id);
void send(info* info_ptr, int id, int mesId);
void get_string(info* info_ptr, int id);
void claimStLucia(info* info_ptr, int id);
int getStLuciaID(info* info_ptr);
void zeroMessages(info* info_ptr);
void reroll(info* info_ptr, int id, char* rollBuffer);
void points(info* info_ptr, int id, int* pts); //updates points for a turn
void run_game(info* info_ptr, char* rollBuffer);
void rollLength(info* info_ptr);
void handle_roll(info* info_ptr, int id, char* rollBuffer);
void handle_attack(info* info_ptr, int id, int* pts);
void heal(info* info_ptr, int id);
void read_stderr(info* info_ptr, int id, int cmd);
void invalidMes(info* info_ptr, int id, int status);
void send_all(info* info_ptr, int notP, int messID);
void report(info* info_ptr, int* pts, int id);
int eliminate(info* info_ptr);
void send_attack(info* info_ptr, int id, int* pts, int c_id);
void check_process(info* info_ptr, int id);


/*          Functions                       */


void init_player(int player, info* info_ptr){
    /* sends child process to work in p_type file "*/
    char str[4];
    sprintf(str, "%d", info_ptr->playerNum);
    char* myID = player_struct[player].playerName;
    char* type = player_struct[player].playerType; 
    dup2(player_struct[player].IN[0], STDIN_FILENO);
    dup2(player_struct[player].OUT[1], STDOUT_FILENO);
    dup2(player_struct[player].ERR[1], STDERR_FILENO);
    close(player_struct[player].OUT[1]);
    close(player_struct[player].ERR[1]);
    
    //execute given player
    execl(type, type, str, myID, NULL);
    //execution failed, printf to stderr and exit
    hubErr(5);
}

void make_roll(info* info_ptr, int player, int rnum, char* rollBuffer){
    /* Makes a roll for a specific player i */
    int i, j=0;
    for( i = rnum; i<(6+rnum); i++){
        player_struct[player].rollStr[j] = rollBuffer[info_ptr->CRN];
        j +=1;
        ++info_ptr->CRN;
        rollLength(info_ptr);
    }
    player_struct[player].rollStr[j] = '\0';

    arrangeP(player);
}


void initialize_pipe(info* info_ptr, char* rollBuffer){
    
    int pnum = info_ptr->playerNum;
    for( int i = 0; i < pnum; i++){
        if (pipe(player_struct[i].IN) == 1){
            hubErr(5);
        }
        if(pipe(player_struct[i].OUT) == 1){
            hubErr(5);
        }
        if(pipe(player_struct[i].ERR) == 1){
            hubErr(5);
        }

        if((player_struct[i].ppid=fork()) == -1){
            hubErr(5);
        }
        if (player_struct[i].ppid == 0){
            ///child process
            close(player_struct[i].IN[1]);  //child will read from parent
            close(player_struct[i].OUT[0]); 
            close(player_struct[i].ERR[0]);
            // send process to player program
            init_player(i, info_ptr); 
        } else{
            ///parent process;
            //set pipes such that parent can't read to stdin and write from
            //stdout or stderr of the player process
            close(player_struct[i].IN[0]);
            close(player_struct[i].OUT[1]);
            close(player_struct[i].ERR[1]);
            
            // check that the child can respond
            check_process(info_ptr, i);
        }

    }
    //childen initialized, parent go to run the game     
    run_game(info_ptr, rollBuffer);
}

void run_game(info* info_ptr, char* rollBuffer){
    //set the game in a loop until a winner is found
    int game = 1;
    while(game){
        //initialize the message to be sent through to the player  
        zeroMessages(info_ptr);    
        int id;
        for(id = 0; (id<info_ptr->playerNum); id++){
            if (info_ptr->playerList[id] == 222){
                //player is not active (eliminated), move to next player
                continue;
            }
            // initialize points for the round, find player in stLucia
            int in = player_struct[id].stLucia;
            int current = getStLuciaID(info_ptr);
            int* pts = malloc(sizeof(int));            
            // give a player a roll, and reroll if needed
            handle_roll(info_ptr, id,rollBuffer);
            // report the attacks for the chosen roll
            int stLucia = getStLuciaID(info_ptr);
            handle_attack(info_ptr, id, pts);
            // report if a new player has claimed stLucia
            if (getStLuciaID(info_ptr) == 222){;}
                /*      no one is in StLucia        */
            else if ((in == 0)&&(player_struct[id].stLucia == 1)){
                fprintf(stderr,"Player %s claimed StLucia\n", 
                player_struct[id].playerName);
                    /*  tell players of new claim     */
                message(info_ptr, 8, id);
                send_all(info_ptr, 222, 0);
                claimStLucia(info_ptr, id);
            } else if ((stLucia != id) && (info_ptr->attack >0)){
                    /*attack stLucia, ask if want to leave*/
                message(info_ptr, 9, id);
                send(info_ptr, stLucia, 0);
                get_string(info_ptr, stLucia);
                /*check if player wanted to leave*/
                if (player_struct[stLucia].playerMes == 4){
                    message(info_ptr, 8, id);
                    send_all(info_ptr, 222, 0);
                    claimStLucia(info_ptr, id);
                    fprintf(stderr,"Player %s claimed StLucia\n", player_struct[id].playerName);
                    pts[0] += 1;
                    player_struct[id].points +=1;
                } else if (player_struct[stLucia].playerMes != 3){
                    hubErr(8);
                }
            }

            if (current == id){
            /* player has been in stLucia for a round, add pounts   */
                pts[0] += 2;
                player_struct[id].points += 2;
            }
            /*   report on points and player eliminations         */
            report(info_ptr, pts, id);
            free(pts);
        }
    }
    exit(0);
}


int main(int argc, char* argv[]){
    /* generates the information struct for the game and required buffers
     * then initalizes piping and starts the game
     * */
    info* info_ptr = makeInfo();
    /*  check program inputs and make players   */
    check_input(argc, argv, info_ptr);
    makeplayer(info_ptr, argv, argc);
    char* rollBuffer = malloc(sizeof(char)*(info_ptr->RFL));
    /* store the contents of the role file in the rollBuffer */
    get_rolls(info_ptr, rollBuffer);
    info_ptr->StLMes = malloc(sizeof(char)*20);
    info_ptr->ERRMes = malloc(sizeof(char)*60);
    /* begin to run the game */
    initialize_pipe(info_ptr, rollBuffer);
    return 0;
}


info* makeInfo(void){
    /* initialize the game's information struct */
    info* info_ptr = malloc(sizeof(info));
    return info_ptr;
}


void handle_roll(info* info_ptr, int id, char* rollBuffer){
    /*        sends a roll to the player until one is chosen  */
    int init_roll = 1;
    while (player_struct[id].playerMes != 1){
        if (init_roll){
            int rnum = info_ptr->CRN;
            make_roll(info_ptr, id, rnum, rollBuffer);
          //  info_ptr->CRN = info_ptr->CRN+6;
            rollLength(info_ptr);
            send(info_ptr, id, 1);
            init_roll = 0;
        } else if(player_struct[id].playerMes == 2){
            reroll(info_ptr, id, rollBuffer);
            send(info_ptr, id, 2);
            rollLength(info_ptr);
        } else {
            hubErr(8);
        }
        get_string(info_ptr, id);
    }
    /* tells other players the roll player id chose     */
    fprintf(stderr,"Player %s rolled %s\n", player_struct[id].playerName, 
        player_struct[id].rollStr);        
        
    message(info_ptr, 3, id);
    send_all(info_ptr, id, 0);
    heal(info_ptr, id);
}

void handle_attack(info* info_ptr, int id, int* pts){
    /*  handles player's attack to stLucia or other players            */
            // get points for turn, this updates attack value
   points(info_ptr, id, pts);
   if (info_ptr->attack > 0 ){ 
        int status = player_struct[id].stLucia;   //in stLucia?
        int stLucia = getStLuciaID(info_ptr);     //ID of player in stLucia
        if (status){
            /*          player in st lucia attacks everyone         */

            for (int j=0; j<(info_ptr->playerNum); j++){
                if ( j == id){
                    continue;
                } else if (info_ptr->playerList[j] != 222){
                    message(info_ptr, 5, id);
                    send_attack(info_ptr, j, pts, id);
                }   
            }
        } else{
                /*          not in st Lucia      */
            if (stLucia == id){
                ;                  //do nothing, no one else is in stLucia
            } else if( stLucia == 222){
                ;                   //no one in stLucia
            } else {    /* attack st Lucia      */
                message(info_ptr, 6, id);
                send_attack(info_ptr, stLucia, pts, id);
            }
        }
    } else if (info_ptr->attack == -1){
        /*       no one was in stLucia and rolled an A -> claim     */
        claimStLucia(info_ptr, id);
        pts[0] += 1;
        player_struct[id].points += 1;

    }

}

void send_attack(info* info_ptr, int id, int* pts, int c_id){
    /* contols the output of attacks    */
    int sL = getStLuciaID(info_ptr);
    int oldHealth = player_struct[sL].health;
    int attack = info_ptr->attack; 
    if (player_struct[id].health < attack){
        /*  player health can't be negative, make it zero   */
          attack = player_struct[id].health;
    }
    /*  inform players of attack    */
    send(info_ptr, id, 0);
    player_struct[id].health -= attack;
    fprintf(stderr,"Player %s took %d damage, health is now %d\n", 
                        player_struct[id].playerName, 
                        attack, player_struct[id].health);
        
    if ((oldHealth > 0)&&(player_struct[sL].health < 1)){
        /*    id has attacked StLucia and won              */ 
        claimStLucia(info_ptr, c_id);
        pts[0] += 1;
        player_struct[c_id].points += 1;
    }

}


void makeplayer(info* info_ptr, char** argv, int argc){
    /*      initializes points for players, and list of active players   */
    int pnum = argc - 3;
    info_ptr->playerNum = pnum;
    for (int j = 0; j<26; j++){
        info_ptr->playerList[j] = 222;  //all players inactive
    }
    for(int i=0; i<pnum; i++){
        player_struct[i].id = i;
        player_struct[i].playerType = argv[i+3];
        player_struct[i].playerName = alpha[i]; //name of player
        info_ptr->playerList[i] = i;    //player i is now active
        player_struct[i].health = 10;   //health initially 10
        player_struct[i].tokens = 0;
        player_struct[i].stLucia = 0;
        for(int j=0; j<6; j++){
            player_struct[i].reroll[j] = '0';//initial reroll is empty/invalid
        }
        player_struct[i].reroll[6] = '\0';
    }
}


void check_process(info* info_ptr, int id){
    /*  checks that an '!' can be retreived from player with id */
    FILE* stream = fdopen(player_struct[id].OUT[0], "r");
    int status;
    waitpid(player_struct[id].ppid, &status, WNOHANG);
    int c = fgetc(stream);
    int stat = 1;
    while (stat < 10){
        if (c != '!'){
            c = fgetc(stream);
            ++stat;
        } else {
            player_struct[id].active = 1;
            stat = 30;
            break;
        }
    }
    if (player_struct[id].active == 0){
        /* '!' wasnt found, raise error */
        hubErr(5);
    }

}


void send(info* info_ptr, int id, int mesId){
    /* creates a stream to send a message to a player */
   if (mesId != 0){
        /* a message wasn't preset, make one */
        message(info_ptr, mesId, id);
   }
   /* write to specified player with id the message  */
   write(player_struct[id].IN[1], info_ptr->StLMes, info_ptr->MesSize);
   /* check that is was received  */
   read_stderr(info_ptr, id, 1);
}


void send_all(info* info_ptr, int notP, int messID){
    /*  send to all avaliable players   */
    if (notP == 222){
        // send to all active players
        for (int i=0; i<26; i++){
            if (info_ptr->playerList[i] != 222){
                send(info_ptr, i, messID);
            }
        }
    } else{
        /*   send to all avaliable players but player notP         */
         for (int i=0; i<26; i++){
            if (info_ptr->playerList[i] != 222){
                if(info_ptr->playerList[i] != notP){
                    send(info_ptr, i, messID);
                }
            }
        }       
    }
}

void read_stderr(info* info_ptr, int id, int cmd){
        /*   check the exit status or stderr of the player       */
    if (cmd){
        /* requested to check exit status  */
        int status;
        waitpid(player_struct[id].ppid, &status, WNOHANG);
        if(WIFEXITED(status)){
            if(WEXITSTATUS(status) != 0){
                /* player exited */
                invalidMes(info_ptr, id, WEXITSTATUS(status));
            }
        }
    } else {
        /* check the stderr of the player   */
        char buffer[1024];
        FILE* error = fdopen(player_struct[id].ERR[0], "r");
        int count = 0;
        int c = fgetc(error);
        /* retrieve stderr */
        do{
            if (c=='!'){ 
                c= fgetc(error); 
                continue;
            }
            buffer[count] = c;
            count += 1;
            if ((c == '\n')||(count > 32)){ 
                break;
            }
            c = fgetc(error);
        } while((c != '\n') || (count < 32));
        int len = strlen(buffer);
        for (int i=0; i<len; i++){
            if ((buffer[i] == 'B')&&(buffer[i+1]=='a')&&(buffer[i+2]=='d')){
                /*Bad message from StLucia received*/
                invalidMes(info_ptr, id, 5);
           }
        }
    }
}


void get_string(info* info_ptr, int id){
    /* creates a stream to get a message from player, then interprets */
    read_stderr(info_ptr, id, 0);
    char buffer[1024];
    FILE* stream = fdopen(player_struct[id].OUT[0], "r");
    int count = 0;
    int c = fgetc(stream);
    /* update buffer with player output until newline */
    do{
        if (c=='!'){
            player_struct[id].active = 1;
            c= fgetc(stream); 
            continue;
        }
        buffer[count] = c;
        count += 1;
        if ((c == '\n')||(count > 24)){ 
            break;
        }
        c = fgetc(stream);
    } while((c != '\n') || (count < 24));
    /* update struct value representing player message */
    if ((buffer[0] == 'g')&&(buffer[1] == 'o')){
        player_struct[id].playerMes = 4;
    } else if ((buffer[0] == 's')&&(buffer[1] == 't')&&
                (buffer[2] == 'a')&&(buffer[3] == 'y')){
        player_struct[id].playerMes = 3;
    } else if ((buffer[0] == 'k')&&(buffer[1] == 'e')&&
                (buffer[2] == 'e')&&(buffer[3] == 'p')&&(buffer[4] == 'a')&&
                (buffer[5] == 'l')&&(buffer[6] == 'l')){
        player_struct[id].playerMes = 1;
    } else if (buffer[0] == 'r'){
        /* possibly requested for reroll, check and get reroll */ 
        char* message = malloc(sizeof(char)*18);
        strcpy(message, buffer);
        char* tokens[2];
        tokens[0] = strtok(message, " ");
        tokens[1] = strtok(NULL, " ");

        if (strcmp(tokens[0], "reroll") == 0){
            fflush(stdout);
            for(int i=0;i<6;i++){
                player_struct[id].reroll[i] = '0';
            }
            player_struct[id].reroll[6] = '\0';
            /*   work out which dice to reroll, update reroll buffer   */
            for (int i = 0; i<6; i++){
                if (tokens[1][i] == '1'){
                    player_struct[id].reroll[i] = '1';
                 }else if (tokens[1][i] == '2'){
                    player_struct[id].reroll[i] = '2';
                }else if (tokens[1][i] == '3'){
                    player_struct[id].reroll[i] = '3';
                }else if (tokens[1][i] == 'H'){
                    player_struct[id].reroll[i] = 'H';
                }else if (tokens[1][i] == 'A'){
                    player_struct[id].reroll[i] = 'A';
                }else if (tokens[1][i] == 'P'){
                    player_struct[id].reroll[i] = 'P';
                }    
            }
        player_struct[id].playerMes = 2;
        }
        free(message);      
    } 
   
    if (player_struct[id].playerMes == 0){
        /*  could not interpret message -> invalid    */
        hubErr(7);
    }
}

void rollLength(info* info_ptr){
    /* checks if appending buffer value out of buffer length to current roll
     * restart buffer count if reached end of buffer
     * */
    if (info_ptr->CRN > info_ptr->RFL){
        info_ptr->CRN = 0;
        
    } else if (info_ptr->CRN == info_ptr->RFL){
        info_ptr->CRN = 0;
    }

}


void reroll(info* info_ptr, int id, char* rollBuffer){
    /* handles player request to reroll by marking dice to keep and returning
     * a new rollStr containing new and kept dice*/
    int size = 6;
    for(int i=0; i<6; i++){
        if (player_struct[id].reroll[i] == '0'){
        /* reroll dice up to invalid dice char '0', gets num dice to reroll */
            size = i;
            break;
        }
    }
    if (size == 6){
        /* requested to reroll all 6 dice, generate a new set */
        make_roll(info_ptr, id, info_ptr->CRN, rollBuffer);
    } else{
        /* update rollStr dice corresponding to dice in reroll buffer*/
        for (int j=0; j< 6; j++){
            for( int k=0; k<size; k++){
                if (player_struct[id].rollStr[j] == player_struct[id].reroll[k]){
                    int buf = info_ptr->CRN;
                    player_struct[id].rollStr[j] = rollBuffer[buf];
                    info_ptr->CRN += 1;
                    rollLength(info_ptr);
                    break;
               }    
          }
       }
    }
    /*   check roll buffer length and sort rollStr   */
    rollLength(info_ptr);
    arrangeP(id);
}

void get_rolls(info* info_ptr, char* rollBuffer){
    /* open file and get bunches of 6 dice rolls and ignore newlines, 
    * add them to buffer */
    char* filename = info_ptr->rollFile;
    FILE* file = fopen(filename, "r");
    if(file == NULL){
        hubErr(3);
    }
    int c;
    int count;
    for(count=0; count < (info_ptr->RFL); count++) {
        int stable = 1;
        while(stable){
            c = fgetc(file);
            if (c == EOF){
                stable = 0;
                break;
            }
            int check = 0;
            if( c != '\n'){
            for (int i=0; i<6; i++){
                if(c == roll[i]){
                    rollBuffer[count] = c;
                    check = 0;
                    stable = 0;
                    break;
                }
            }}
            if (check == 1){
                hubErr(4);
            }
        
            if(stable == 0){
                break;
            }
        } 
    }
}

void arrangeP(int player){
    /* Arranges the dice roll for a given player */
    char* rollStr = player_struct[player].rollStr;
    char arrStr[7];
    int i = 0;
    for(int e = 0; e<6; e++){
        for(int r = 0; r < 6; r++){
            if(rollStr[r] == roll[e]){
                arrStr[i] = rollStr[r];
                ++i;
            }
        }
    }
    arrStr[7] = '\0';
    for(int j=0; j<7; j++){
        player_struct[player].rollStr[j] = arrStr[j];
    }
}

char* arrange(char* rollStr){
    /* Arranges a given diceroll and returns */
    char* arrStr = malloc(sizeof(char)*7);
    int i = 0;
    for(int e = 0; e<6; e++){
        for(int r = 0; r < 6; r++){
            if(rollStr[r] == roll[e]){
                arrStr[i] = rollStr[r];
                ++i;
            }
        }
    }
    arrStr[7] = '\0';
    return arrStr;
}


void report(info* info_ptr, int* pts, int id){

    //if the current player scored points, printf to stderr
    if (pts[0] > 0){
        fprintf(stderr,"Player %s scored %d for a total of %d\n",
            player_struct[id].playerName, pts[0], player_struct[id].points);
    }
    //update who was eliminated and playerList
    int playerLeft = eliminate(info_ptr);
    if (playerLeft != 222){
        message(info_ptr, 10, playerLeft);
        send_all(info_ptr, 222, 0);
        fprintf(stderr, "Player %s wins\n", player_struct[playerLeft].playerName);
        exit(0);
    }
    //tell players and report to stderr if a player has won the game
    for (int u=0; u < info_ptr->playerNum; u++){
        if (player_struct[u].points >= info_ptr->MAX){
             message(info_ptr, 10, u);
             send_all(info_ptr, 222, 0); 
             fprintf(stderr,"Player %s wins\n", player_struct[u].playerName);
             exit(0);
             break;
        }
    }
}


int eliminate(info* info_ptr){
    /*   returns the id of the last player in the game, otherwise return 
     *   invalid player 222 */
    // check all players health, if they are eliminated update active list
    for (int x=0; x<info_ptr->playerNum; x++){
        if ((player_struct[x].health < 1)&&(info_ptr->playerList[x] != 222)){
            message(info_ptr, 7, x);
            send_all(info_ptr, 222, 0);
            //printf("eliminating : %s\n", player_struct[x].playerName);     
            player_struct[x].stLucia = 0; 
            info_ptr->playerList[x] = 222;
        }
    }
    // calculate how many players are left incase second last player was elim
    int players =0;
    int id = 222;
    for (int y=0; y<26; y++){
        if (info_ptr->playerList[y] != 222){
            ++players;
        }
    }
    if (players < 2){
        for (int z=0; z<26; z++){
            if (info_ptr->playerList[z] != 222){
                id = z;
            }
            
        }

    }
    return id;
}

void claimStLucia(info* info_ptr, int id){
    /* updates for all players, 0 for player not in stLucia 
     *  1 for player with id, as they have now claimed stLucia*/
    for (int i=0; i<(info_ptr->playerNum); i++){
        player_struct[i].stLucia = 0;
    }
    player_struct[id].stLucia = 1;
}

int getStLuciaID(info* info_ptr){
    /* searches all players t determine which is in stLucia,
     * if no one is, invalid player 222 is returned*/
    for (int i=0; i<(info_ptr->playerNum); i++){
        if (player_struct[i].stLucia == 1){
        //    printf("%d in Stlucia\n", player_struct[i].id);
            return player_struct[i].id;

        }
    }
    return 222;

}

void heal(info* info_ptr, int id){    
    /* searches player id's roll and updates health accordingly, 
     * doesn't change health if they are current in stLucia */
    int health = 0, heal=0;
    char* mess = malloc(sizeof(char)*20);
    strcpy(mess, info_ptr->StLMes);
    char* token2 = strtok(mess, " ");
    token2 = strtok(NULL, " ");
    char* token3 = strtok(NULL, " ");
    if  (player_struct[id].playerName[0] == token2[0]){
        for (int i=0; i<6; i++){
            if ((player_struct[id].health < 10)&&(token3[i] == 'H')&&
                    (!player_struct[id].stLucia)){
                player_struct[id].health += 1;
                health += 1;
            }
            if (token3[i] == 'H'){
                heal += 1;
            }
        }
    } 
    if (((player_struct[id].health == 10)&&(heal)&&
        (!player_struct[id].stLucia))||(health > 0)){
         fprintf(stderr,"Player %s healed %d, health is now %d\n",
            player_struct[id].playerName, health, player_struct[id].health);
    }
    free(mess);
}

void points(info* info_ptr, int id, int* pts){
    /* examines player id's roll to update points and tokens */
    info_ptr->attack = 0;
    char* roll = malloc(sizeof(char)*7);
    strcpy(roll, player_struct[id].rollStr);
    int ones=0, twos=0, threes=0, H=0, A=0, P=0, heal=0;
    int points = 0;
    pts[0] = 0;

    for (int i=0; i<6; i++){
        if (roll[i] == '1'){
            ++ones;
        }else if (roll[i] == '2'){
            ++twos;
        }else if (roll[i] == '3'){
            ++threes;
        }else if (roll[i] == 'H'){
            ++H;
        }else if (roll[i] == 'A'){
            ++A;
        }else if (roll[i] == 'P'){
            ++P;
        }
    }
    if (ones > 2){
        pts[0] += (ones-2);
        points += (ones-2);
    }
    if (twos >2){
        pts[0] += (2+(twos-3));
        points += (2+(twos-3));
    }
    if (threes >2){
        pts[0] += (3+(threes-3));
        points += (3+(threes-3));
    }
    if (A){
        if (getStLuciaID(info_ptr) == 222){
            /* invalid attack to symbolize A was rolled and no player holds
             * stLucia*/
            info_ptr->attack = -1;
        } else{
            /* if stLucia is claimed, update attack value */
            info_ptr->attack += A;
        }
    }
    if (P){
        /* update amount of tokens */
        player_struct[id].tokens += P;
    }
    if (player_struct[id].tokens > 9){
        /* takes 10 tokens from player and gain 1 health */
        pts[0] += 1;
        player_struct[id].tokens = player_struct[id].tokens % 10;
    }

    free(roll);
    player_struct[id].health += heal;
    if (player_struct[id].health > 10){
        player_struct[id].health = 10;
    }
    player_struct[id].points += pts[0];
}

void zeroMessages(info* info_ptr){
    /* initalize player messages to be 0, these will change to non-zero values
     * when requested to send a message to stLucia. playerMes contains
     * the message id, representing a valid message to send to a player*/
    for (int i=0; i<(info_ptr->playerNum); i++){
        player_struct[i].playerMes = 0;
    }
}


void message(info* info_ptr, int cond, int id){
    /* updates the message to be sent to player id with the message matching
     * the cond argument*/
    switch(cond){
        case 1: 
            sprintf(info_ptr->StLMes, "turn %s\n", player_struct[id].rollStr);
            //strcpy(mess, "turn");
            //strcat(mess, p);
            //info_ptr->StLMes = mess
            break;
        case 2:
            sprintf(info_ptr->StLMes, "rerolled %s\n", player_struct[id].rollStr);
            break;
        case 3:
            sprintf(info_ptr->StLMes, "rolled %s %s\n", player_struct[id].playerName, player_struct[id].rollStr);
            break;
        case 4:
            sprintf(info_ptr->StLMes, "points %s %d\n",
                        player_struct[id].playerName, player_struct[id].points);
            break;
        case 5:

            sprintf(info_ptr->StLMes, "attacks %s %d in\n",
                        player_struct[id].playerName, info_ptr->attack);
            break;
        case 6:
            sprintf(info_ptr->StLMes, "attacks %s %d out\n",
                        player_struct[id].playerName, info_ptr->attack);
            break;
        case 7:
            sprintf(info_ptr->StLMes, "eliminated %s\n", 
                        player_struct[id].playerName);
            break;
        case 8:
            sprintf(info_ptr->StLMes, "claim %s\n", 
                        player_struct[id].playerName);
            break;
        case 9:
            strcpy(info_ptr->StLMes, "stay?\n");
            break;
        case 10:
            sprintf(info_ptr->StLMes, "winner %s\n", player_struct[id].playerName);
            break;
        case 11:
            strcpy(info_ptr->StLMes, "shutdown\n");
            break;
    }

    info_ptr->MesSize = strlen(info_ptr->StLMes);
}

void ERRMessage(int status){
    /* contains possible error messages for the hub to send to stderr
     * before exiting */
    char cat1[100], cat2[50];
    const char* ERRMes;
    switch(status){
        case 1: 
            //char cat1[32], cat2[28];
            strcpy(cat1, "Usage: stlucia rollfile winscore");
            strcpy(cat2, " prog1 prog2 [prog3 [prog4]]\n");
            strcat(cat1, cat2);
            ERRMes = cat1;
            break;
        case 2:
            ERRMes = "Invalid score\n";
            break;
        case 3:
            ERRMes = "Unable to access rollfile\n";
            break;
        case 4:
            ERRMes = "Error reading rolls\n";
            break;
        case 5:
            ERRMes = "Unable to start subprocess\n";
             break;
        case 6:
            ERRMes = "Player quit\n";
            break;
        case 7:
            ERRMes = "Invalid message received from player\n";
            break;
        case 8:
            ERRMes = "Invalid request by player\n";
            break;
        case 9:
            ERRMes = "SIGINT caught\n";
            break;
    }
    fprintf(stderr, ERRMes);
}

void hubErr(int ext){
 /* sends an error message to stderr */
    ERRMessage(ext);
    exit(ext);
}

void invalidMes(info* info_ptr, int id, int status){
    hubErr(6);
}

void check_file(char* filename, info* info_ptr){
 /* checks the dice file, adds first instance to buffer */
  
    FILE* file = fopen(filename, "r");
    if(file == NULL){
        hubErr(3);
    }
    info_ptr->RFL = 0;    
    int c;
    c = fgetc(file);
    int count = 0;
    while (c != EOF) {
        int status = 1;
        if(c == '\n'){
            c = fgetc(file);
            continue;
        }
        if(c == EOF){
            break;
        } 
        for(int i=0; i<6; i++){
            if (c == roll[i]){ 
               // info_ptr->rollStr[i] = c;
                info_ptr->RFL += 1;
                status = 0;
                break;
            }
        }
        if(status != 0){
            hubErr(4);
        }
        c = fgetc(file);
        count++;
    }    
    fclose(file);
    fflush(file);
}

void check_input(int argc, char* argv[], info* info_ptr){
    /* check the arguments to the program */
    if (argc < 5){
        hubErr(1);
    }
    info_ptr->rollFile = argv[1];
    check_file(info_ptr->rollFile, info_ptr);
    //check that the maxscore is a valid positive integer
    char scoreLen = strlen(argv[2]);
    char nums[10] = {'1','2','3','4','5','6','7','8','9','0'};
    int test = 0;
    for (int i=0; i<scoreLen; i++){
        for (int j=0; j<10; j++){
            if (argv[2][i] == nums[j]){
                ++test;
                break;
            }
        }
    }
    if (scoreLen != test){
        hubErr(2);
    }
    char* longerr;
    info_ptr->playerNum = argc - 3;
    info_ptr->MAX = strtol(argv[2], &longerr, 10);

    if (info_ptr->playerNum == 0){
        hubErr(1);
    }
}



