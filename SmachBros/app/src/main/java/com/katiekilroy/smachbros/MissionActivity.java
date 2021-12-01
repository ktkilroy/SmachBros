package com.katiekilroy.smachbros;

import android.content.Intent;
import android.os.Bundle;
import android.view.View;

import androidx.appcompat.app.AppCompatActivity;

public class MissionActivity extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_mission);
    }

    public void onBeginClicked(View view) {
        Intent newIntent = new Intent(this, LevelActivity.class);
        startActivity(newIntent);
    }
}