package com.katiekilroy.smachbros;

import android.content.Context;

import org.json.JSONException;
import org.json.JSONObject;

import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;

public class json {

    public static JSONObject toJsonString(int maneuver, int distance) {
        JSONObject jsonObject = new JSONObject();
        try {
            jsonObject.put("Transition", String.valueOf(maneuver));
            jsonObject.put("Distance",  String.valueOf(distance));
        } catch (JSONException e) {
            e.printStackTrace();
        }
        return jsonObject;
    }

    public static void writeToFile(Context context, String filename, String str){

        try{
            FileOutputStream fos = context.openFileOutput(filename, Context.MODE_PRIVATE);
            fos.write(str.getBytes(), 0, str.length());
            fos.close();
        }
        catch (IOException e){
            e.printStackTrace();
        }
    }

    public static void save(Context context, String jsonString) throws IOException {
        File rootFolder = context.getExternalFilesDir(null);
        File jsonFile = new File(rootFolder, "file.json");
        FileWriter writer = new FileWriter(jsonFile);
        writer.write(jsonString);
        writer.close();
    }
}



