package com.katiekilroy.smachbros;

import android.content.Intent;
import android.os.Bundle;
import android.view.View;

import androidx.appcompat.app.AppCompatActivity;

public class LevelActivity extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_level);
    }

    public void onMissionClicked(View view) {
        Intent newIntent = new Intent(this, MissionActivity.class);
        startActivity(newIntent);
    }

    public void onFreestyleClicked(View view) {
        Intent newIntent = new Intent(this, FreestyleActivity.class);
        startActivity(newIntent);
    }
}