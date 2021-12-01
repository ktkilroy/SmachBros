package com.katiekilroy.smachbros;

import android.os.Bundle;
import android.os.Environment;
import android.view.View;
import android.widget.EditText;
import android.widget.NumberPicker;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;

import org.json.JSONException;

import java.io.BufferedOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.Socket;

import static android.widget.Toast.LENGTH_LONG;

//public class FreestyleActivity extends AppCompatActivity implements AdapterView.OnItemSelectedListener {

public class FreestyleActivity extends AppCompatActivity {
    private int pos;
    private TextView tw;
    private String[] values;
    private String value;
    static final int SocketServerPORT = 10901;
    private EditText et;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_freestyle);
        NumberPicker np = (NumberPicker) findViewById(R.id.np);
        tw = findViewById(R.id.textView3);
        et = findViewById(R.id.editTextNumber);

        //Initializing a new string array with elements
        values = new String[]{"Forward", "Reverse", "Right", "Left", "Stop", "Flip"};
        np.setMinValue(0);
        np.setMaxValue(values.length - 1);
        np.setDisplayedValues(values);
        np.setWrapSelectorWheel(true);
        np.setOnValueChangedListener(new NumberPicker.OnValueChangeListener() {
            @Override
            public void onValueChange(NumberPicker picker, int oldVal, int newVal) {
                //getting values from spinner
                pos = newVal;
                value = values[newVal];
            }
        });
    }

    private class ClientRxThread extends Thread {
        String dstAddress;
        int dstPort;

        ClientRxThread(String address, int port) {
            dstAddress = address;
            dstPort = port;
        }


        @Override
        public void run() {
            Socket socket = null;

            try {
                socket = new Socket(dstAddress, dstPort);

                File file = new File(
                        Environment.getExternalStorageDirectory(),
                        "plan.json");

                byte[] bytes = new byte[1024];
                InputStream is = socket.getInputStream();
                FileOutputStream fos = new FileOutputStream(file);
                BufferedOutputStream bos = new BufferedOutputStream(fos);
                int bytesRead = is.read(bytes, 0, bytes.length);
                bos.write(bytes, 0, bytesRead);
                bos.close();
                socket.close();


//                        Toast.makeText(this, "Finished", Toast.LENGTH_LONG).show();

            } catch (IOException e) {

                e.printStackTrace();

                final String eMsg = "Something wrong: " + e.getMessage();
//                Toast.makeText(this, eMsg, LENGTH_LONG).show();


            } finally {
                if (socket != null) {
                    try {
                        socket.close();
                    } catch (IOException e) {
                        // TODO Auto-generated catch block
                        e.printStackTrace();
                    }
                }
            }

//            Toast.makeText("hello", this,
//                    LENGTH_LONG).show();
        }
    }


    public void writeToJson(String filename, String content) {
        File path = getApplicationContext().getFilesDir();
        try {
            FileOutputStream writer = new FileOutputStream(new File(path, filename));
            writer.write(content.getBytes());
//            Toast.makeText(this, "hello",
//                    LENGTH_LONG).show();

        } catch (IOException e) {
            e.printStackTrace();
        }

    }

    public String readFromFile(String filename) {
        File path = getApplicationContext().getFilesDir();
        File readFrom = new File(path, filename);
        byte[] content = new byte[(int) readFrom.length()];
        try {
            FileInputStream stream = new FileInputStream(readFrom);
            stream.read(content);
            return new String(content);
        } catch (Exception e) {
            e.printStackTrace();
            return e.toString();
        }
    }

    public void onSubmitClicked(View view) throws IOException, JSONException {

        String distance = String.valueOf(et.getText());
        if (distance.matches("")){
            Toast.makeText(this, "Input a value for distance!",
                    LENGTH_LONG).show();
        }
        else {
            if (pos == 0) tw.append("\nForward for " + distance + "m\n");
            else tw.append("\n" + value + " for " + distance + "m\n");

            String userString = "\"type\":\"freestyle\",\n\"maneuvers\": [ \n { \n\"transition\" : \"" +
                    pos +
                    "\", \n \"parameter\" : \"" + distance + "\"\n}\n]";

            writeToJson("plan.json", userString);

            String content = readFromFile("plan.json");
            tw.append(content);

            ClientRxThread clientRxThread =
                    new ClientRxThread("IP ADDRESS",
                            SocketServerPORT);

            clientRxThread.start();
            et.getText().clear();
        }
    }
}