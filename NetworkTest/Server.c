// Imports
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <time.h> 

// Function declarations
void printHeadersIntoFile();

int main(int argc, char *argv[]) {
    int listenfd = 0, connfd = 0;
    int n;
    struct sockaddr_in serv_addr;
    char receiveBuff[1025];

    // Set up server
    listenfd = socket(AF_INET, SOCK_STREAM, 0);
    memset(&serv_addr, '0', sizeof(serv_addr));
    memset(receiveBuff, '0', sizeof(receiveBuff)); 

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_addr.sin_port = htons(5000); 

    bind(listenfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)); 

    // Listen for clients
    listen(listenfd, 10); 

    printHeadersIntoFile();

    while(1) {
	// Receive a client
        connfd = accept(listenfd, (struct sockaddr*)NULL, NULL);

	// Remove unused space from buffer
	while ( (n = read(connfd, receiveBuff, sizeof(receiveBuff)-1)) > 0) {
        	receiveBuff[n] = 0;
    	}

	// Print received data to console
	printf("\n%s\n", receiveBuff);

	// Open the data file
	FILE *dataFile = fopen("data.csv", "a");

	// Check file can be opened
	if (dataFile == NULL) {
	    printf("Error opening file!\n");
	    exit(1);
	}

	// Print row of sensor data to file
	fprintf(dataFile, "%s\n", receiveBuff);

	// Close the file
	fclose(dataFile);

	// Disconnect client
        close(connfd);
        sleep(1);
     }
}

 /**
  * Prints a heading to describe which columns correspond to which sensor readings
  */
void printHeadersIntoFile() {
	// Open the file for writing
	FILE *dataFile = fopen("data.csv", "w");

	// Write header
	fprintf(dataFile, "Time Stamp (ms),Temperature (*C),Pressure (Pa),AccelX (m/s^2),AccelY (m/s^2),AccelZ (m/s^2)");
	fprintf(dataFile, "MagX (uT),MagY (uT),MagZ (uT),GyroX (rad/s),GyroY (rad/s),GyroZ (rad/s),OriPitch,OriRoll");
	fprintf(dataFile, "OriHeading,Latitude,Longitude,Altitude (m),Velocity (m/s)\n");

	// Close the file
	fclose(dataFile);
}
