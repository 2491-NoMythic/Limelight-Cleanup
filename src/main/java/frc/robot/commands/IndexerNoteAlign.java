// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.settings.Constants.IndexerConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IndexerNoteAlign extends SequentialCommandGroup {
  IndexerSubsystem indexer;
  IntakeSubsystem intake;
  
  /** Creates a new indexerNoteAlign. 
   * This is a sequential command group that will run automatically in robot container when we detect that we have a note.
   * it runs the note backwards in the indexer until the sensor no longer detects a note and then runs the note forward //use with toggleOnTrue to stop when we detect the note again
   * this will hopefully make our note placement within the indexer more consistant.
   * */
  public IndexerNoteAlign(IndexerSubsystem indexer, IntakeSubsystem intake) {
    this.intake = intake;
    this.indexer = indexer;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()->indexer.magicRPS(-5), indexer),
      new WaitUntil(()->!intake.isNoteSeen()),
      new InstantCommand(()->indexer.magicRPS(5), indexer),
      new WaitUntil(()->intake.isNoteSeen()),
      new InstantCommand(()->intake.setNoteHeld(true))
    );
  }
}