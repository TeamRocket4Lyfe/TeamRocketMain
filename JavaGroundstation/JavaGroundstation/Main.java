import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.Socket;

public class Main {

    public static void main(String argv[]) throws Exception {

        final String ipAddress = "192.168.1.77";
        final int port = 8888;

        System.out.println("Listening...");

        try {

            while (true) {
                Socket socket = new Socket(ipAddress, port);
                BufferedReader reader = new BufferedReader(new InputStreamReader(socket.getInputStream()));
                System.out.println("recieved:");
                while(!reader.ready());
                String line = reader.readLine();
                System.out.println(line);
                reader.close();
            }

        } catch (IOException ex) {
            ex.printStackTrace();
        }

    }   // mainn()

}   // Main
