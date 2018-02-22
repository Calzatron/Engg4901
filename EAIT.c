#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

typedef struct P{
    int game;           //is 1 if the game is being run
    int playerNum;      //the amount of players in the game
    int reroll;         //amount of times the player has rerolled (max 2)
    int health;         //health of the player
    int StLucia;        //whether or not the player is in StLucia (1)
    int result;         //integer representing the message from StLucia
    int mLen;           //the length of the current mesage from StLucia
    char* myID;         //player distinction between A->Z
    char* sendRoll;     //roll's to reroll/send back
    char* keepRoll;     //dice to keep when sorting a roll
    char rollStr[6];    //player's kept dice roll
    int ppid;           //parent process id on check
    int ppid1;          //parent process id on start up
} P;


/*    array of possible player names      */
const char* alpha[26] = {"A","B","C","D","E","F","G","H","I","J","K","L","M",
                        "N","O","P","Q","R","S","T","U","V","W","X","Y","Z"};
/*  possible dice rolls    */
const char roll[6] = {'1','2','3','H','A','P'};

/*    function initializing player struct      */
P* makeP(void){
    P* p_ptr = malloc(sizeof(P));
    return p_ptr;
}

/*  function declarations    */
void turn(char buffer[18], P* p_ptr);
void interpret(char* message, P* p_ptr);
int choose_roll(P* p_ptr, char* buffer);
int eliminate(P* p_ptr, char buffer[18]);
void stay(P* p_ptr);
void claim(char* buffer, P* p_ptr);
void attacks(char buffer[18], P* p_ptr);
void health(P* p_ptr, char* buffer);
void run_player(P* p_ptr, char* buffer);
void input(P* p_ptr, char* buffer);
void rolls(P* p_ptr, char* buffer);
void player_check(P* p_ptr, char* buffer);
void value_check(P* p_ptr, char* buffer);
void length_check(P* p_ptr, char* buffer);
void connection(P* p_ptr);



void check_args(int argc, char** argv, P* p_ptr){
    /*  checks arguments to the player, returns the 
     *  appropriate exit status and message if invalid  */
    //      incorrect amount of arguments
    if (argc != 3){
        fprintf(stderr, "Usage: player number_of_players my_id\n");
        exit(1);
    }
    //      try to get the amount of players in the game
    char* errorMes;
    p_ptr->playerNum = strtol(argv[1], &errorMes, 10);
    if((p_ptr->playerNum < 2)||(errorMes[0] != '\0')){
        fprintf(stderr, "Invalid player count\n");
        exit(2);
    }
    //      get the player ID, if invalid exit due to error
    int status = 1;
    for(int i=0; i<26; i++){
        if ((argv[2][0] == alpha[i][0]) && (argv[2][1] == alpha[i][1])){
            p_ptr->myID = argv[2];
            status = 0;
            break;
        }
    }
    if(status == 1){
        fprintf(stderr, "Invalid player ID\n");
        exit(3);
    }

}

void connection(P* p_ptr){
    /* check that the parent id is the same as at start up
     * if a different value is returned from getppid(),
     * lost connection to parent process */
    p_ptr->ppid = getppid();
    if (p_ptr->ppid1 != p_ptr->ppid){
        fprintf(stderr,"Unexpectedly lost contact with StLucia\n");
        exit(4);
    }
}

int main(int argc, char** argv){
    /* initializes the program to allow player to run correctly */
    P* p_ptr = makeP();
    check_args(argc, argv, p_ptr);
    //      show that argument checking completed
    p_ptr->ppid1 = getppid();
    printf("!");
    fflush(stdout);
    //      initialize player variables
    char* buffer = malloc(sizeof(char)*18);
    p_ptr->sendRoll = malloc(sizeof(char)*7);
    p_ptr->keepRoll = malloc(sizeof(char)*7);
    p_ptr->game = 1;
    p_ptr->health = 10;
    p_ptr->StLucia = 0;
    //      run the player
    run_player(p_ptr, buffer);
    return 0;
}

void run_player(P* p_ptr, char* buffer){
    /*  deal with message from stLucia until shutdown or this player
     *  is eliminated */
    while(p_ptr->game){
        connection(p_ptr);
        input(p_ptr, buffer);
        if ((p_ptr->result > 11)||(p_ptr->result < 0)){
            fprintf(stderr, "Bad message from StLucia %s\n",buffer);
            exit(5);
        } else if ((p_ptr->result == 1)||(p_ptr->result == 2)){
            rolls(p_ptr, buffer);
        } else if (p_ptr->result == 3){
            player_check(p_ptr, buffer);
            continue;
        } else if (p_ptr->result == 4){
            value_check(p_ptr, buffer);
            player_check(p_ptr, buffer);
        } else if ((p_ptr->result == 10)||eliminate(p_ptr, buffer)
                                        ||(p_ptr->result == 9)){
            exit(0);
            p_ptr->game = 0;
            continue;
        } else if (p_ptr->result == 7){
            player_check(p_ptr, buffer);
            claim(buffer, p_ptr);
        } else if (p_ptr->result == 5){
            attacks(buffer, p_ptr);
        } else if (p_ptr->result == 8){
            stay(p_ptr);   
        } else {
            p_ptr->game = 1;
            continue;
        }
        fflush(stdout);
    }
}

void input(P* p_ptr, char* buffer){
    /* get the input from StLucia and add it to the buffer to be interpreted */
    //initialise value representing interpretation, buffer and roll
    p_ptr->result = 0;
    memset(buffer, 0, 18);
    p_ptr->rollStr[0] = '0';
    memset(p_ptr->rollStr, 0, 6);
    //get response up to a new line or first 18 characters
    int count = 0;
    int c = fgetc(stdin);
    while((c != '\n') || count < 18){
        buffer[count] = c;
        ++count;
        c = fgetc(stdin);
        if (c == EOF){
            fprintf(stderr,"Unexpectedly lost contact with StLucia\n");
            exit(4);
        } else if (c == '\n'){
            break;
        }
    }
    // interpret the message
    interpret(buffer, p_ptr);
    // check if the message is a suitable length for it's interpretation
    length_check(p_ptr, buffer);
}


void interpret(char* buffer, P* p_ptr){
    /*      interprets the message from stLucia
     *      updates 'result' with the determined message */
    //copy the buffer to allow comparison without destroying its contents
    connection(p_ptr);
    char* message = malloc(sizeof(char)*18);
    strcpy(message, buffer);
    //      report to stLucia through stderr the message received
    char* send = malloc(sizeof(char)*30);
    sprintf(send, "From StLucia:%s\n", message);
    fprintf(stderr, send);
    free(send);
    p_ptr->mLen = strlen(message);
    //      break down the message into strings and determine result
    //      value by key words
    if (strcmp(message, "shutdown") == 0){
        //  stLucia has shutdown, terminate now, by exit and manually (incase)
        exit(0);
        p_ptr->game = 0;
        p_ptr->result = 11;
    } else if (strcmp(message, "stay?") == 0){
        p_ptr->result = 8;
    } else {
        char* space = " ";
        char* firstStr = strtok(message, space);
        char* possMes[8] = {"turn", "rerolled", "rolled", "points", "attacks",
                            "eliminated", "claim", "winner"};
        for (int i=1; i< 9; i++){
            if (strcmp(firstStr, possMes[i-1]) == 0){
                if (i == 8){
                    //  a player has won, close the player
                   ++i;
                   p_ptr->result = i;
                   player_check(p_ptr, buffer);
                   exit(0);
                }
                if (i==6){
                    player_check(p_ptr, buffer); 
                }
                p_ptr->result = i;
                break;
            }
        }
    }
    if (p_ptr->result == 0){
        fprintf(stderr, "Bad message from StLucia\n");
        exit(5);
    }
    free(message);
}


void rolls(P* p_ptr, char* buffer){
    /*   function for choosing a roll, will loop until one is chosen
     *   reinitializes the keep roll buffer after use */
    int choosing = 1;
    while(choosing){
        turn(buffer, p_ptr);
        choosing = choose_roll(p_ptr, buffer);
        fflush(stdout);
        if(choosing){
           input(p_ptr, buffer); 
        } else {
            memset(p_ptr->keepRoll, 0, 7);
            p_ptr->reroll = 0;
        }

    }   

}

void attacks(char buffer[18], P* p_ptr){
    /*      deals with StLucia reporting a player has attacked 
     *      and updates the player's health if they were attacked */
    //  check that the player and point value are valid
    value_check(p_ptr, buffer);
    player_check(p_ptr, buffer);
    char* mess = malloc(sizeof(char)*18);
    // copy the buffer to allow extraction
    strcpy(mess, buffer);
    char v[2] = {(mess[10]),'\0'};
    char* err;
    int points = strtol(v, &err, 10);
    if(mess[8] == p_ptr->myID[0]){
        // player cannot attack itself, do nothing
        ;
    } else if ((mess[12] == 'i')&&(mess[13] == 'n')){
        // if player from in stlucia attacking, lose health
            p_ptr->health = p_ptr->health - points;
    } else if ((mess[12] == 'o')&&(mess[13] == 'u')&&(mess[14] == 't')){
        // if i am in st lucia and being attacked from out, lose health
                if(p_ptr->StLucia == 1){
                    p_ptr->health = p_ptr->health - points;
                }
    } else {
        fprintf(stderr, "Bad message from StLucia\n");// %s\n",mess);

        exit(5);
    }
    free(mess);
}




void claim(char* buffer, P* p_ptr){
    /*  determines if stLucia reported if this player has claimed stLucia*/

    char* mess = malloc(sizeof(char)*18);
    strcpy(mess, buffer);
    if (mess[6] == p_ptr->myID[0]){
       //this player claimed StLucia
       p_ptr->StLucia = 1;
    } else{
       //this player is not in StLucia
        p_ptr->StLucia = 0;
    }
    free(mess);
}


void stay(P* p_ptr){
    /*     respond to being asked if player wants to stay in stLucia       */
    //only treat if health is less than 5
    if(p_ptr->health < 5){
        p_ptr->StLucia = 0;
        printf("go\n");
    } else{
        p_ptr->StLucia = 1;
        printf("stay\n");
    }
}



void turn(char buffer[18], P* p_ptr){
    /*    checks the input buffer and roll                   */

    int check = 1, len=0, match = 0;
    char turn[5] = {'t','u','r','n','\0'};
    char rerolled[9] = {'r','e','r','o','l','l','e','d','\0'};
    // check if was a valid message
    for (int i=0; i<4; i++){
        if ((buffer[i] != turn[i])&&(buffer[i] != rerolled[i])){
            check = 0;
            break;
        }
     }
     if (buffer[0] == 't'){
        len = 5;
     } else {
        len = 9;
     }
     // check rolls are valid and append to roll string so point updates
     // and reroll choice can be performed
     int total = 0;
     for (int i = 0; i<6; i++){
         for (int j=0; j<6; j++){
             if (buffer[i+len] == roll[j]){
                p_ptr->rollStr[i] = roll[j];
                match = 1;
                ++total;
                break;
             }
         }
     }
     if ((match != 1)||(total != 6)||(check !=1)){
        //  interpreted message as a new roll but it wasnt -> bad message */
         fprintf(stderr, "Bad message from StLucia\n");
         exit(5);
     }
}



int choose_roll(P* p_ptr, char* buffer){
    /*  determines which rolls to reroll and keep, also updates health  */
    //reinitialize the roll buffer to send back to StLucia
    memset(p_ptr->sendRoll, 0, 6);
    //buffer containing position of rolls to keep, keep pos -> 1
    int index[6] = {0,0,0,0,0,0};
    int ones = 0, twos=0, threes=0, H=0, i;
    for (i=0; i<6; i++){
        if (p_ptr->rollStr[i] == '1'){
            ++ones;
        }
        if (p_ptr->rollStr[i] == '2'){
            ++twos;
        }
        if (p_ptr->rollStr[i] == '3'){
            ++threes;
        }
        if ((p_ptr->health < 6)&&(p_ptr->rollStr[i] == 'H')){
            ++H;
        }
    }
    /*      mark the location of rolls we want to keep as 1 in index    */
    if (ones > 2){
        for (i=0;i<6;i++){
            if(p_ptr->rollStr[i] == '1'){
                index[i] = 1;
            }
        }
    }
    if (twos > 2){
        for (i=0;i<6;i++){
            if(p_ptr->rollStr[i] == '2'){
                index[i] = 1;
            }
        }
    }
    if (threes > 2){
        
        for (i=0;i<6;i++){
            if(p_ptr->rollStr[i] == '3'){
                index[i] = 1;
            }
        }


    }
    if (H){
        for (i=0;i<6;i++){
            if(p_ptr->rollStr[i] == 'H'){
                index[i] = 1;
            }
        }

    }
    /*               filter through roll                             */
    int j = 0, k = strlen(p_ptr->keepRoll);
    for (i = 0; i<6; i++){
        if(index[i] == 0){
            /*      if it wasn't mark, append to send back           */
            p_ptr->sendRoll[j] = p_ptr->rollStr[i];
            ++j;
        }
    }
            /*      can only reroll twice -> keep                   */
    if (p_ptr->reroll > 1){
        i = 0;
        for(k = strlen(p_ptr->keepRoll); k<(6); k++){
            p_ptr->keepRoll[k] = p_ptr->rollStr[i];
            ++i;
        }
        printf("keepall\n");
        health(p_ptr, buffer);
        return 0;
    } else if ((j == 0)|| (k == 6)){
            /* if there is no need to reroll, update points */
        printf("keepall\n");
        health(p_ptr, buffer);
        return 0;
    }    
    printf("reroll %s\n", p_ptr->sendRoll);
    ++p_ptr->reroll;
            /* otherwise, we want to reroll */
    return 1;
}

void health(P* p_ptr, char* buffer){
    /* updates health depending if player is in stLucia and number
     * of H's rolled after keepall          */

    char* mess = malloc(sizeof(char)*20);
    strcpy(mess, buffer);
    char* token1 = strtok(mess, " ");
    token1 = strtok(NULL, " ");
    for (int i=0; i<6; i++){
        if ((p_ptr->health <10)&&(token1[i] == 'H')&&
                    (!p_ptr->StLucia)){
                
            p_ptr->health += 1;
            
        }
    }               
}

int eliminate(P* p_ptr, char* buffer){
    /* compares the elimated command from stLucia with the current player*/
    char iAmEliminated[14];
    sprintf(iAmEliminated, "eliminated %s", p_ptr->myID);
    if (strcmp(buffer, iAmEliminated) == 0){
        return 1;
    }
    return 0;
}

void player_check(P* p_ptr, char* buffer){
    /* checks that the message received from StLucia contains a valid
     * player name*/
    char* mess = malloc(sizeof(char)*20);
    strcpy(mess, buffer);
    //ensure the breaks are in the correct places for the given message
     if ((p_ptr->result == 3) || (p_ptr->result == 4)){
        if ((mess[6] != ' ')||(mess[8] != ' ')||(mess[9] == '\n')){ 
                fprintf(stderr, "Bad message from StLucia\n");
            exit(5);
        }
    } else if (p_ptr->result == 5){
        if ((mess[7] != ' ')||(mess[9] != ' ')||(mess[11] != ' ')||
                (mess[12]=='\n')){
            fprintf(stderr, "Bad message from StLucia\n");
            exit(5);
        }
    }
    //split the buffer at the space and check the player name
    char* token = strtok(mess, " ");
    token = strtok(NULL, " ");
    int num = p_ptr->playerNum;
    int i=0;
    for (i=0; i<26; i++){
        if (token[0] == alpha[i][0]){
            break;
        }
    }
    // ensure the index of the player name is within the amount of players
    // in the game
    if (i >= num){
       fprintf(stderr, "Bad message from StLucia\n");
       exit(5);
    }    
    free(mess);
}

void value_check(P* p_ptr, char* buffer){
    /* checks that the ammount of points in the message is a valid
     * postive integer */
    char* mess = malloc(sizeof(char)*20);
    strcpy(mess, buffer);
    char* token = strtok(mess, " ");
    token = strtok(NULL, " ");
    token = strtok(NULL, " ");
    char nums[10] = {'1','2','3','4','5','6','7','8','9','0'};
    int test = 0;
    int size = strlen(token);
    for (int j=0; j<size; j++){
        for (int i=0; i<10; i++){
            if(token[j] == nums[i]){
                test += 1;
            }
        }    
    }
    if (test != size){
        fprintf(stderr, "Bad message from StLucia\n");
        exit(5);
    }
    free(mess);
}


void length_check(P* p_ptr, char* buffer){
    /*   check the length of the current message to ensure it is valid
     *   if it isnt, StLucia has sent a bad message*/
    if (((p_ptr->result == 1)&&(p_ptr->mLen != 11))||
        ((p_ptr->result == 2)&&(p_ptr->mLen != 15))||
        ((p_ptr->result == 3)&&(p_ptr->mLen != 15))||
        ((p_ptr->result == 4)&&(p_ptr->mLen < 10))||
        ((p_ptr->result == 6)&&(p_ptr->mLen != 12))||
        ((p_ptr->result == 7)&&(p_ptr->mLen != 7))||
        ((p_ptr->result == 8)&&(p_ptr->mLen != 5))){
            fprintf(stderr, "Bad message from StLucia\n");
            exit(5);
        
    }
}














