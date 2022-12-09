// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

package com.eprosima.idl.parser.strategy;

import com.eprosima.log.ColorMessage;
import com.eprosima.idl.parser.exception.ParseException;
import org.antlr.v4.runtime.Parser;
import org.antlr.v4.runtime.RecognitionException;
import org.antlr.v4.runtime.NoViableAltException;
import org.antlr.v4.runtime.InputMismatchException;
import org.antlr.v4.runtime.FailedPredicateException;
import org.antlr.v4.runtime.Token;
import org.antlr.v4.runtime.misc.NotNull;
import org.antlr.v4.runtime.misc.Nullable;

public class DefaultErrorStrategy extends org.antlr.v4.runtime.DefaultErrorStrategy
{
    public static final DefaultErrorStrategy INSTANCE = new DefaultErrorStrategy();

    @Override
    public void reportError(Parser recognizer, RecognitionException e)
    {
        if (inErrorRecoveryMode(recognizer)) {
            // System.err.print("[SPURIOUS] ");
            return; // don't report spurious errors
        }
        beginErrorCondition(recognizer);
        if ( e instanceof NoViableAltException ) {
            reportNoViableAlternative(recognizer, (NoViableAltException) e);
        }
        else if ( e instanceof InputMismatchException ) {
            reportInputMismatch(recognizer, (InputMismatchException)e);
        }
        else if ( e instanceof FailedPredicateException ) {
            reportFailedPredicate(recognizer, (FailedPredicateException)e);
        }
        else if ( e instanceof ParseException) {
            if(e.getOffendingToken() != null)
            {
                String message = ColorMessage.bold(getTokenErrorDisplay(e.getOffendingToken()) + " ") + e.getMessage();
                recognizer.notifyErrorListeners(e.getOffendingToken(), message, e);
            }
            else
                recognizer.notifyErrorListeners(e.getMessage());
        }
        else {
            System.err.println("unknown recognition error type: "+e.getClass().getName());
            recognizer.notifyErrorListeners(e.getOffendingToken(), e.getMessage(), e);
        }
    }

    @Override
    protected void reportUnwantedToken(@NotNull Parser recognizer)
    {
        if (inErrorRecoveryMode(recognizer))
        {
            return;
        }

        beginErrorCondition(recognizer);
        Token t = recognizer.getCurrentToken();
        String tokenName = getTokenErrorDisplay(t);
        String msg = "Unexpected input " + ColorMessage.bold(tokenName);
        recognizer.notifyErrorListeners(t, msg, null);
    }
}
